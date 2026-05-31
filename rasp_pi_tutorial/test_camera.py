#!/usr/bin/env python3
"""
Camera diagnostic — run this to find which index and resolution
the Astra Pro Plus delivers live frames on.

Usage:
    python3 test_camera.py
"""
import cv2

RESOLUTIONS = [
    (1920, 1080),
    (1280, 720),
    (640, 480),
]

def test_index(index: int) -> None:
    cap = cv2.VideoCapture(index)
    if not cap.isOpened():
        print(f"  index {index}: cannot open")
        return

    # Warm up
    for _ in range(5):
        cap.read()

    for w, h in RESOLUTIONS:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)

        ret, frame = cap.read()
        if not ret or frame is None:
            print(f"  index {index} @ {w}×{h}: read failed")
            continue

        actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        mean_val = float(frame.mean())
        status = "OK (has content)" if mean_val > 1.0 else "BLACK (no content)"
        print(f"  index {index} @ {actual_w}×{actual_h}: mean={mean_val:.1f}  {status}")

        if mean_val > 1.0:
            # Show the first working frame so you can see it
            cv2.imshow(f"index {index} {actual_w}x{actual_h}", frame)
            cv2.waitKey(2000)
            cv2.destroyAllWindows()

    cap.release()


print("Scanning camera indices 0-9...")
for i in range(10):
    test_index(i)

print("\nDone. Use the index + resolution that showed 'OK (has content)'.")
