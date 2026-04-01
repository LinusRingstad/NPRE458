import cv2
import numpy as np

def nothing(x):
    pass

# 0 should be usb spec
cap = cv2.VideoCapture(0) 
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

cv2.namedWindow('Calibration')

cv2.createTrackbar('StartY', 'Calibration', 48, 720, nothing)
cv2.createTrackbar('Height', 'Calibration', 25, 200, nothing)
cv2.createTrackbar('StartX', 'Calibration', 0, 1280, nothing)
cv2.createTrackbar('Width', 'Calibration', 723, 1280, nothing)

print("Adjust sliders until the green box captures the spectrum.")
print("Press 'q' to print your new coordinates and exit.")

while True:
    ret, frame = cap.read()
    if not ret:
        break

    sy = cv2.getTrackbarPos('StartY', 'Calibration')
    sh = cv2.getTrackbarPos('Height', 'Calibration')
    sx = cv2.getTrackbarPos('StartX', 'Calibration')
    sw = cv2.getTrackbarPos('Width', 'Calibration')

    # Draw the box
    debug_frame = frame.copy()
    cv2.rectangle(debug_frame, (sx, sy), (sx + sw, sy + sh), (0, 255, 0), 2)
    
    cv2.imshow('Calibration', debug_frame)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        print(f"\n--- NEW COORDINATES ---")
        print(f"StartY: {sy}")
        print(f"SizeY:  {sh}")
        print(f"StartX: {sx}")
        print(f"Width:  {sw} (EndX would be {sx + sw})")
        break

cap.release()
cv2.destroyAllWindows()