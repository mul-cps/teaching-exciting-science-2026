"""Optional CPU GG-CNN pixel proposals; no world pose or pickup confidence.

Architecture adapted from https://github.com/dougsm/ggcnn (Douglas Morrison,
2018, BSD-3-Clause; see assets/GGCNN-LICENSE). Cornell-trained proposals need
physical validation and RGB-D/base registration before arm commands.

Install CPU PyTorch separately; importing this module does not import torch.
Depth input units are explicit (default: uint16 millimetres, scale=0.001).
ROI is a square (left, top, right, bottom), right/bottom exclusive. Positive
image angle is counterclockwise: contact direction = (cos(angle), -sin(angle)).
"""
from __future__ import annotations

import argparse
from dataclasses import dataclass
import hashlib
import io
import json
from pathlib import Path, PurePosixPath
import tempfile
import urllib.request
import zipfile

import numpy as np

MODEL_SIZE = 300
CHECKPOINT_URL = 'https://github.com/dougsm/ggcnn/releases/download/v0.1/ggcnn_weights_cornell.zip'
ARCHIVE_SHA256 = '465ef506f1fad68cf1ce461fbda3bbe0686ab9ed221e25db8e87b803c9527514'
STATE_MEMBER = 'ggcnn_weights_cornell/ggcnn_epoch_23_cornell_statedict.pt'


@dataclass(frozen=True)
class PreparedDepth:
    image: np.ndarray
    valid: np.ndarray
    object_mask: np.ndarray
    roi: tuple[int, int, int, int]


def prepare_depth(depth, roi, *, depth_scale=.001, object_mask=None):
    """Resize square depth crop, fill holes for inference, retain exclusion mask.

    Nearest-neighbour valid/object masks prevent interpolation across holes
    from making unsupported grasp centres eligible. Infilled pixels never
    enter the mean used for normalization.
    """
    import cv2
    depth = np.asarray(depth)
    if depth.ndim != 2 or not np.isfinite(depth_scale) or depth_scale <= 0:
        raise ValueError('Expected 2D depth and positive finite depth_scale')
    if len(roi) != 4 or any(int(v) != v for v in roi):
        raise ValueError('ROI must contain four integer bounds')
    x0, y0, x1, y1 = map(int, roi)
    if not (0 <= x0 < x1 <= depth.shape[1] and 0 <= y0 < y1 <= depth.shape[0]) or x1-x0 != y1-y0:
        raise ValueError('ROI must be square and inside the depth image')
    crop = depth[y0:y1, x0:x1].astype(np.float32) * depth_scale
    valid = np.isfinite(crop) & (crop > 0)
    if not valid.any():
        raise ValueError('ROI has no valid depth')
    crop[~valid] = np.median(crop[valid])
    # OpenCV inpaint follows upstream GG-CNN hole filling, without skimage.
    if not valid.all():
        scale = float(np.max(np.abs(crop)))
        crop = cv2.inpaint(crop / scale, (~valid).astype(np.uint8), 1, cv2.INPAINT_NS) * scale
    image = cv2.resize(crop, (MODEL_SIZE, MODEL_SIZE), interpolation=cv2.INTER_LINEAR)
    resized_valid = cv2.resize(valid.astype(np.uint8), (MODEL_SIZE, MODEL_SIZE), interpolation=cv2.INTER_NEAREST).astype(bool)
    image = np.clip(image - np.mean(image[resized_valid]), -1., 1.).astype(np.float32)
    if object_mask is None:
        resized_object = np.ones_like(resized_valid)
    else:
        mask = np.asarray(object_mask, dtype=bool)
        if mask.shape != depth.shape:
            raise ValueError('object_mask must have the full depth image shape')
        resized_object = cv2.resize(mask[y0:y1, x0:x1].astype(np.uint8), (MODEL_SIZE, MODEL_SIZE), interpolation=cv2.INTER_NEAREST).astype(bool)
    return PreparedDepth(image, resized_valid, resized_object, (x0, y0, x1, y1))


def _architecture():
    # Keep torch optional and local. Layer names match the published state dict.
    import torch.nn as nn
    import torch.nn.functional as functional

    class GGCNN(nn.Module):
        def __init__(self):
            super().__init__()
            self.conv1 = nn.Conv2d(1, 32, 9, stride=3, padding=3)
            self.conv2 = nn.Conv2d(32, 16, 5, stride=2, padding=2)
            self.conv3 = nn.Conv2d(16, 8, 3, stride=2, padding=1)
            self.convt1 = nn.ConvTranspose2d(8, 8, 3, stride=2, padding=1, output_padding=1)
            self.convt2 = nn.ConvTranspose2d(8, 16, 5, stride=2, padding=2, output_padding=1)
            self.convt3 = nn.ConvTranspose2d(16, 32, 9, stride=3, padding=3, output_padding=1)
            self.pos_output = nn.Conv2d(32, 1, 2)
            self.cos_output = nn.Conv2d(32, 1, 2)
            self.sin_output = nn.Conv2d(32, 1, 2)
            self.width_output = nn.Conv2d(32, 1, 2)

        def forward(self, x):
            for layer in (self.conv1, self.conv2, self.conv3, self.convt1, self.convt2, self.convt3):
                x = functional.relu(layer(x))
            return self.pos_output(x), self.cos_output(x), self.sin_output(x), self.width_output(x)

    return GGCNN()


def load_model(weights):
    """Load only a tensor state dictionary on CPU, never arbitrary pickle code."""
    import torch
    model = _architecture()
    state = torch.load(Path(weights), map_location='cpu', weights_only=True)
    model.load_state_dict(state, strict=True)
    model.eval()
    return model


def download_weights(cache_dir=None):
    """Download hash-pinned archive and write only its named state-dict member.

    The archive's pickled full-model member is never extracted or loaded.
    Existing cache files are refreshed from the verified archive, rather than
    trusting a path merely because its filename matches.
    """
    cache_dir = Path(cache_dir or Path.home() / '.cache/cps-grasp-models')
    with urllib.request.urlopen(CHECKPOINT_URL, timeout=30) as response:
        archive = response.read(32 * 1024 * 1024 + 1)
    if len(archive) > 32 * 1024 * 1024 or hashlib.sha256(archive).hexdigest() != ARCHIVE_SHA256:
        raise ValueError('GG-CNN archive SHA256 mismatch or oversized download')
    with zipfile.ZipFile(io.BytesIO(archive)) as bundle:
        for info in bundle.infolist():
            name = PurePosixPath(info.filename)
            if name.is_absolute() or '..' in name.parts or '\\' in info.filename:
                raise ValueError('Unsafe archive member name')
        info = bundle.getinfo(STATE_MEMBER)
        if info.file_size > 4 * 1024 * 1024:
            raise ValueError('Oversized state dictionary')
        state_bytes = bundle.read(info)
    cache_dir.mkdir(parents=True, exist_ok=True)
    target = cache_dir / 'cornell-state.pt'
    with tempfile.NamedTemporaryFile(dir=cache_dir, delete=False) as temporary:
        temporary.write(state_bytes)
        temp_path = Path(temporary.name)
    try:
        # Validate tensor keys/shapes before publishing the cache entry.
        load_model(temp_path)
        temp_path.replace(target)
    finally:
        temp_path.unlink(missing_ok=True)
    return target


def candidates_from_maps(quality, angle, width, prepared, *, top_n=5, min_distance=20, min_quality=.2):
    """Select valid masked quality peaks; invert crop for centres and contacts.

    width is in 300x300 network pixels. quality_score is a network score, not a
    calibrated confidence or a probability of successful physical pickup.
    """
    import cv2
    if top_n < 0 or min_distance < 1 or not np.isfinite(min_quality):
        raise ValueError('Invalid peak selection settings')
    quality, angle, width = (np.asarray(a) for a in (quality, angle, width))
    if any(a.shape != (MODEL_SIZE, MODEL_SIZE) for a in (quality, angle, width)):
        raise ValueError('Prediction maps must be 300x300')
    x0, y0, x1, y1 = prepared.roi
    scale = (x1 - x0) / MODEL_SIZE
    grid_row, grid_col = np.ogrid[:MODEL_SIZE, :MODEL_SIZE]
    center_x = x0 + (grid_col+.5)*scale-.5
    center_y = y0 + (grid_row+.5)*scale-.5
    half_width = width * (.5 * scale)
    delta_x = half_width * np.cos(angle)
    delta_y = -half_width * np.sin(angle)
    # Reject both contacts on all pixels before ranking or any Python loop.
    contact_inside = ((center_x-np.abs(delta_x) >= x0)
                      & (center_x+np.abs(delta_x) <= x1-1)
                      & (center_y-np.abs(delta_y) >= y0)
                      & (center_y+np.abs(delta_y) <= y1-1))
    allowed = (prepared.valid & prepared.object_mask & contact_inside
               & np.isfinite(quality) & np.isfinite(angle) & np.isfinite(width)
               & (width > 0) & (quality >= min_quality))
    if top_n == 0 or not allowed.any():
        return []
    masked = np.where(allowed, quality, -np.inf).astype(np.float32)
    maxima = cv2.dilate(masked, np.ones((3, 3), np.uint8))
    peak_scores = np.where(allowed & (masked == maxima), masked, -np.inf)
    selected = []
    # Exactly one vectorized global maximum/suppression pass per output, rather
    # than Python-scanning up to 90,000 rejected peaks on a quality plateau.
    for _ in range(top_n):
        index = int(np.argmax(peak_scores))
        row, col = divmod(index, MODEL_SIZE)
        if not np.isfinite(peak_scores[row, col]):
            break
        theta = float(angle[row, col])
        opening = float(width[row, col]) * scale
        center = np.array([center_x[0, col], center_y[row, 0]])
        delta = np.array([delta_x[row, col], delta_y[row, col]])
        contacts = np.stack((center-delta, center+delta))
        selected.append({'center_xy': center.tolist(), 'angle_radians': theta, 'opening_pixels': opening, 'contacts_xy': contacts.tolist(), 'quality_score': float(quality[row, col])})
        suppressed = (grid_row-row)**2 + (grid_col-col)**2 < min_distance**2
        peak_scores[suppressed] = -np.inf
    return selected


def predict(model, prepared, *, top_n=5, min_distance=20, min_quality=.2):
    """CPU inference with Gaussian maps and periodic grasp-axis smoothing.

    Unlike legacy upstream scalar-angle smoothing, doubled-angle sine/cosine
    filtering preserves the equivalence of grasp axes at +90 and -90 degrees.
    """
    import cv2
    import torch
    with torch.inference_mode():
        tensor = torch.from_numpy(prepared.image).reshape(1, 1, MODEL_SIZE, MODEL_SIZE)
        outputs = model(tensor)
        pos, cos, sin, width = (value.numpy().squeeze() for value in outputs)
    quality = cv2.GaussianBlur(pos, (0, 0), 2., borderType=cv2.BORDER_REPLICATE)
    doubled_angle = np.arctan2(sin, cos)
    smooth_sin = cv2.GaussianBlur(np.sin(doubled_angle), (0, 0), 2., borderType=cv2.BORDER_REPLICATE)
    smooth_cos = cv2.GaussianBlur(np.cos(doubled_angle), (0, 0), 2., borderType=cv2.BORDER_REPLICATE)
    angle = np.arctan2(smooth_sin, smooth_cos) / 2.
    opening = cv2.GaussianBlur(width * 150., (0, 0), 1., borderType=cv2.BORDER_REPLICATE)
    return candidates_from_maps(quality, angle, opening, prepared, top_n=top_n, min_distance=min_distance, min_quality=min_quality)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--depth', type=Path, required=True, help='2D .npy depth image')
    parser.add_argument('--weights', type=Path, help='Trusted tensor state dictionary')
    parser.add_argument('--download-weights', action='store_true', help='Fetch SHA256-pinned Cornell state')
    parser.add_argument('--roi', type=int, nargs=4, required=True, metavar=('LEFT', 'TOP', 'RIGHT', 'BOTTOM'))
    parser.add_argument('--depth-scale', type=float, default=.001)
    parser.add_argument('--object-mask', type=Path, help='Optional full-frame boolean .npy mask')
    parser.add_argument('--top-n', type=int, default=5)
    parser.add_argument('--threads', type=int, default=4)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    if args.threads < 1:
        parser.error('--threads must be positive')
    weights = download_weights() if args.download_weights else args.weights
    if weights is None:
        parser.error('Specify --weights or --download-weights')
    import torch
    torch.set_num_threads(args.threads)
    depth = np.load(args.depth, allow_pickle=False)
    mask = np.load(args.object_mask, allow_pickle=False) if args.object_mask else None
    prepared = prepare_depth(depth, args.roi, depth_scale=args.depth_scale, object_mask=mask)
    result = {'model': 'GG-CNN Cornell', 'coordinates': 'full-image pixels, x=column y=row', 'physical_pickup_validated': False, 'roi': list(prepared.roi), 'candidates': predict(load_model(weights), prepared, top_n=args.top_n)}
    encoded = json.dumps(result, indent=2)
    if args.output:
        args.output.write_text(encoded + '\n')
    else:
        print(encoded)


if __name__ == '__main__':
    main()
