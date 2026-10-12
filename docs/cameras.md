# Camera plan: GoPro HERO10 Black first, UVC fallback

## Decision

Use **one fixed overhead camera** and a **stopped conveyor**. Start with 640×480-ish CPU OpenCV HSV segmentation inside a fixed pickup rectangle. Stable illumination, neutral non-red/blue belt, and a pickup guide are more valuable than a large neural model. Two additional USB cameras are optional and deliberately not part of the baseline.

## HERO10 wired USB: supported control path, but unvalidated capture path

GoPro's official **Open GoPro** docs list HERO10 Black (firmware 1.10+) and show that USB exposes an HTTP control interface; the Python SDK (`open-gopro`) includes a `gopro-webcam` command that starts and views a wired stream with OpenCV. This **does not establish** that HERO10 is an ordinary Linux `/dev/video0` UVC camera. Do not assume `cv2.VideoCapture(0)` opens the GoPro directly. GoPro documentation also has **model-specific restrictions for USB webcam streaming**, so verify the exact HERO10 hardware/firmware combination before writing an adapter.

**Instructor trial** (no robot motion):

```bash
python -m venv .venv && source .venv/bin/activate
pip install -e '.[gopro,camera]'
gopro-webcam --help
gopro-webcam
```

1. Power the GoPro; install current compatible firmware; connect data-capable USB-C cable.
2. Verify a GoPro USB network interface appears on Linux (`ip addr`); avoid concurrent proprietary GoPro Webcam software.
3. Run `gopro-webcam` and confirm live view, frame continuity, thermal stability and reliable reconnect.
4. The official [GoPro Python webcam demo source](https://github.com/gopro/OpenGoPro/blob/main/demos/python/sdk_wireless_camera_control/open_gopro/demos/gui/webcam.py) shows `WiredGoPro.streaming.start_stream(StreamType.WEBCAM, WebcamStreamOptions(protocol=WebcamProtocol.TS))`. Its [video reader](https://github.com/gopro/OpenGoPro/blob/main/demos/python/sdk_wireless_camera_control/open_gopro/demos/gui/video_display.py) uses OpenCV with FFmpeg to read the network video URL. This is a practical documented **Python route**, not ordinary UVC webcam enumeration.
5. **Unverified two-terminal test:** run `gopro-webcam --protocol TS` in terminal A and leave it waiting at the "Press Enter" prompt. In terminal B try:

   ```bash
   python tools/camera_probe.py 'udp://@:8554?overrun_nonfatal=1&fifo_size=50000000'
   python -m workshop --camera 'udp://@:8554?overrun_nonfatal=1&fifo_size=50000000' --cycles 3
   ```

   Do not press Enter in A while terminal B is receiving frames; two viewers can conflict on the same UDP port. Confirm the printed URL (which may vary) against the local SDK, and treat this as a **bench test only**, not a verified HERO10 integration.
6. **Pending:** verify a reliable source, decoder support and restart procedure on actual hardware before the instructor signs off.
7. Test for at least 30 minutes at teaching-room lighting and record actual camera address/device/format in acceptance log.

The demo is the simplest *supported diagnostic*, not a validated plug-and-play integration. Wi-Fi control/streaming is another Open GoPro option, but is not necessary for the beginner workshop.

## Guaranteed teaching alternative: standard UVC camera

A conventional USB UVC webcam generally exposes `/dev/video*` on Linux and works directly with OpenCV:

```bash
pip install -e '.[camera]'
python tools/camera_probe.py 0
python -m workshop --camera 0 --cycles 3
```

For a prerecorded source: `python -m workshop --camera recording.mp4 --cycles 3` (works if the local OpenCV build can decode the file). For multiple cameras, test source 0 then 1 individually; do not add stereo fusion.

## Design constraints

Do not pick moving objects. Tune the fixed region in `workshop/vision.py` and use flat 20–30 mm tokens, not loose M&M's. Place a matte neutral background under the pickup area. Keep continuous USB power and avoid the GoPro overheating. A failed camera read MUST prevent robot motion. Never start physical motion from the generic camera-only commands.

## Future (not for student setup)

A Jetson Thor can host an optional instructor-only YOLO-World/YOLOE demo. Do not require it for sorting red and blue objects. On ordinary laptops, basic color masking has deterministic CPU cost and no network/model downloads.
