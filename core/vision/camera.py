from pathlib import Path

import cv2

CAMERA_DEVICE_PATH = Path(
    "/dev/v4l/by-id/usb-046d_Logitech_Webcam_C920-C_D0C829BF-video-index0"
)

def _resolved_device_path():
    if CAMERA_DEVICE_PATH.exists():
        return str(CAMERA_DEVICE_PATH.resolve())
    return None


def _report_settings(cap, source):
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = cap.get(cv2.CAP_PROP_FPS)
    fourcc = int(cap.get(cv2.CAP_PROP_FOURCC))
    fourcc_str = "".join(chr((fourcc >> (8 * i)) & 0xFF) for i in range(4)).strip()
    print(f"Camera source: {source}")
    print(f"Camera mode: {width}x{height} @ {fps:.1f} fps ({fourcc_str or 'unknown'})")


def open_configured_camera():
    device_path = _resolved_device_path()
    if device_path is not None:
        cap = cv2.VideoCapture(device_path, cv2.CAP_V4L2)
        source = str(CAMERA_DEVICE_PATH)
    else:
        cap = cv2.VideoCapture(0)
        source = "camera index 0 (fallback)"

    if not cap.isOpened():
        print(f"Error: could not open camera ({source}).")
        return None

    _report_settings(cap, source)
    return cap
