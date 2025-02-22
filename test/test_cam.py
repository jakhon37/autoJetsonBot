import cv2

cap = cv2.VideoCapture(11)  # or try 1, 2, etc.
if not cap.isOpened():
    print("Could not open camera!")
else:
    ret, frame = cap.read()
    if ret:
        print("Captured a frame!")
    else:
        print("Failed to capture frame")
    cap.release()
