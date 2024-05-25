import cv2
import numpy as np

# Initialize cameras
camera1 = cv2.VideoCapture("inference/Videos/Sample_Video.mp4")
camera2 = cv2.VideoCapture("inference/Videos/Sample_Video - Trim.mp4")  # Adjust index based on the number of cameras

if not camera1.isOpened() or not camera2.isOpened():
    print("Error: Could not open one of the cameras.")
    exit()

while True:
    ret1, frame1 = camera1.read()
    ret2, frame2 = camera2.read()
    
    if not ret1 or not ret2:
        print("Error: Could not read frame from one of the cameras.")
        break

    # Resize frames for better fit if necessary
    frame1 = cv2.resize(frame1, (640, 480))
    frame2 = cv2.resize(frame2, (640, 480))

    # Combine frames horizontally
    combined_frame = np.hstack((frame1, frame2))
    
    # Combine frames vertically
    # combined_frame = np.vstack((frame1, frame2))

    # Display the combined frame
    cv2.imshow('Multi-Camera Feed', combined_frame)

    # Exit on 'q' key press
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

camera1.release()
camera2.release()
cv2.destroyAllWindows()
