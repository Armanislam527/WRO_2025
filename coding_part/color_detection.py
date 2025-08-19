import cv2
import numpy as np

# Initialize webcam
cap = cv2.VideoCapture(2, cv2.CAP_V4L2)
if not cap.isOpened():
    print("Error: Could not open webcam")
    exit()

# Set lower resolution for performance
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))

# Define HSV color ranges (adjust for WRO objects)
colors = {
    'red': ([0, 120, 70], [10, 255, 255], [170, 120, 70], [180, 255, 255]),
    'green': ([40, 40, 40], [80, 255, 255]),
    'blue': ([100, 100, 100], [140, 255, 255])
}

while True:
    ret, frame = cap.read()
    if not ret:
        print("Error: Failed to capture frame")
        break

    # Convert to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Detect each color
    for color, (lower1, upper1, lower2, upper2) in colors.items():
        if color == 'red':
            # Red spans hue 0-10 and 170-180
            mask1 = cv2.inRange(hsv, np.array(lower1), np.array(upper1))
            mask2 = cv2.inRange(hsv, np.array(lower2), np.array(upper2))
            mask = cv2.bitwise_or(mask1, mask2)
        else:
            mask = cv2.inRange(hsv, np.array(lower1), np.array(upper1))

        # Count non-zero pixels (indicating color presence)
        count = cv2.countNonZero(mask)
        if count > 1000:  # Threshold for detection
            print(f"{color.capitalize()} detected (pixels: {count})")

        # Save mask for debugging
        cv2.imwrite(f"/home/pizero/mask_{color}.jpg", mask)

    # Exit on 'q' (for non-headless testing)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Cleanup
cap.release()
cv2.destroyAllWindows()
