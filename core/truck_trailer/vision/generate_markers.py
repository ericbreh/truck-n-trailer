from pathlib import Path
import cv2

MARKERS_DIR = Path(__file__).parent / "markers"

from truck_trailer.vision.config import (
    ARUCO_DICT,
    TRACKING_MARKER_IDS,
    REFERENCE_MARKER_IDS,
)

def save_marker(marker_id: int, label: str) -> None:
    img = cv2.aruco.generateImageMarker(ARUCO_DICT, marker_id, 200)
    filename = MARKERS_DIR / f"marker_{marker_id:02d}_{label.replace(' ', '_')}.png"
    cv2.imwrite(str(filename), img)
    print(f"  Saved: {filename.name}  (ID {marker_id})")


def main() -> None:
    MARKERS_DIR.mkdir(parents=True, exist_ok=True)

    for marker_id, name in TRACKING_MARKER_IDS.items():
        save_marker(marker_id, name)

    for marker_id, name in REFERENCE_MARKER_IDS.items():
        save_marker(marker_id, name)

    total = len(TRACKING_MARKER_IDS) + len(REFERENCE_MARKER_IDS)
    print(f"\nDone. {total} markers written to: {MARKERS_DIR}")


if __name__ == "__main__":
    main()
