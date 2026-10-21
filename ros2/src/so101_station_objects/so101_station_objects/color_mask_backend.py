"""Optional OpenCV OpenCL masks; downstream geometry remains on the CPU.

Construct on the ROS thread and call only from the serial analysis worker.
OpenCV's OpenCL selection is thread-local and initialized lazily, never at import.
An execution failure permanently selects CPU for this instance and current frame.
"""
from __future__ import annotations

import cv2
from tools.propose_rgbd_grasp import color_masks


class ColorMaskBackend:
    def __init__(self, backend='cpu'):
        if backend not in ('cpu', 'vulkan', 'opencl'):
            raise ValueError('color mask backend must be cpu, vulkan or opencl')
        self.requested = backend
        self._selected = 'cpu' if backend == 'cpu' else None
        self._reason = None
        self._device = None
        self._capable = None

    def _initialize(self):
        if self.requested == 'vulkan':
            from .vulkan_color_masks import VulkanColorMasks
            self._vulkan = VulkanColorMasks()
            self._device = self._vulkan.info
            self._capable = True
            self._selected = 'vulkan'
            return
        if not cv2.ocl.haveOpenCL():
            raise RuntimeError('OpenCV has no available OpenCL platform')
        cv2.ocl.setUseOpenCL(True)
        if not cv2.ocl.useOpenCL():
            raise RuntimeError('OpenCV OpenCL could not be enabled')
        device = cv2.ocl.Device_getDefault()
        if not device.type() & cv2.ocl.DEVICE_TYPE_GPU:
            raise RuntimeError('OpenCL default device is not a GPU')
        self._device = {'name': device.name(), 'vendor': device.vendorName(),
                        'version': device.version()}
        self._capable = True
        self._selected = 'opencl'

    def _opencl_masks(self, rgb):
        if not cv2.ocl.useOpenCL():
            raise RuntimeError('OpenCL disabled in analysis thread')
        # One upload and one shared HSV conversion. All four masks remain on
        # device until their final downloads. OpenCV pools temporary buffers;
        # no application cache can retain stale data across shapes or frames.
        hsv = cv2.cvtColor(cv2.UMat(rgb), cv2.COLOR_RGB2HSV)
        masks = {
            'red': cv2.bitwise_or(cv2.inRange(hsv, (0, 90, 50), (10, 255, 255)),
                                  cv2.inRange(hsv, (170, 90, 50), (180, 255, 255))),
            'green': cv2.inRange(hsv, (41, 90, 50), (89, 255, 255)),
            'blue': cv2.inRange(hsv, (90, 90, 50), (140, 255, 255)),
            'yellow': cv2.inRange(hsv, (15, 90, 50), (40, 255, 255)),
        }
        return {color: mask.get().astype(bool) for color, mask in masks.items()}

    def __call__(self, rgb):
        if self._selected != 'cpu':
            try:
                if self._selected is None:
                    self._initialize()
                return self._vulkan(rgb) if self._selected == 'vulkan' else self._opencl_masks(rgb)
            except Exception as error:
                self._reason = f'{type(error).__name__}: {error}'[:512]
                if self._capable is None:
                    self._capable = False
                self._selected = 'cpu'
        return color_masks(rgb)

    def status(self):
        return {'requested': self.requested, 'selected': self._selected,
                'gpu_available': self._capable, 'device': self._device,
                'fallback_reason': self._reason}
