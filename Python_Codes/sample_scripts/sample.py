import cv2
from ultralytics import YOLO
import cvzone
import numpy as np
import pandas as pd

model = YOLO("weights/yolov8n.pt")
# model = YOLO("weights/yolov8m.pt")
# model = YOLO("weights/yolov8l.pt")

def RGB(event, x, y, flags, param):
    if event == cv2.EVENT_MOUSEMOVE:
        point = [x, y]
        print(point)

cv2.namedWindow("RGB")
cv2.setMouseCallback("RGB", RGB)

VIDEO_PATH = "inference/Videos/iloveyouVirus.mp4"
# VIDEO_PATH = "inference/Videos/SAMPLE_VIDEO.mp4"

cap = cv2.VideoCapture(VIDEO_PATH)

my_file = open("utils/coco.txt", "r")
data = my_file.read()
class_list = data.split("\n")

count = 0
frame_width = 1020
frame_height = 500
#         TL           BL          BR          TR
#         x     y
area1 = [(371, 223), (374, 340), (648, 340), (644, 218)]
area2 = [(142, 220), (142, 340), (358, 340), (355, 219)]

while True:
    success, frame = cap.read()

    if not success:
        cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
        continue
    count += 1
    if count % 3 != 0:
        continue
    frame = cv2.resize(frame, (frame_width, frame_height))
    results = model.predict(frame)

    a = results[0].boxes.data
    px = pd.DataFrame(a).astype("float")
    
    list2 = []
    list1 = []
    for index, row in px.iterrows():
        x1 = int(row[0])
        y1 = int(row[1])
        x2 = int(row[2])
        y2 = int(row[3])

        d = int(row[5])
        c = class_list[d]

        cx = int(x1 + x2) // 2
        cy = int(y1 + y2) // 2
        w, h = x2 - x1, y2 - y1

        result = cv2.pointPolygonTest(np.array(area1, np.int32), ((cx, cy)), False)
        if result >= 0:
            # cv2.rectangle(frame, (x3, y3), (x4, y4), (0, 255, 0), -1)
            cvzone.cornerRect(frame, (x1, y1, w, h), 3, 2)
            cv2.circle(frame, (cx, cy), 4, (255, 0, 0), -1)
            cvzone.putTextRect(frame, f"person", (x1, y1), 1, 1)
            list1.append(cx)

        result1 = cv2.pointPolygonTest(np.array(area2, np.int32), ((cx, cy)), False)
        if result1 >= 0:
            # cv2.rectangle(frame, (x3, y3), (x4, y4), (0, 255, 0), -1)
            cvzone.cornerRect(frame, (x1, y1, w, h), 3, 2)
            cv2.circle(frame, (cx, cy), 4, (255, 0, 255), -1)
            cvzone.putTextRect(frame, f"person", (x1, y1), 1, 1)
            list2.append(cx)

    cr1 = len(list1)
    cr2 = len(list2)

    cv2.polylines(frame, [np.array(area1, np.int32)], True, (0, 0, 255), 2)
    cv2.polylines(frame, [np.array(area2, np.int32)], True, (0, 255, 0), 2)
    cvzone.putTextRect(frame, f"Middle Zone: {cr1} Left Zone: {cr2}", (50, 60), 1, 1)
    # cvzone.putTextRect(frame, f"", (50, 160), 2, 2)

    cv2.imshow("RGB", frame)

    # if cv2.waitKey(1) == ord("q"):
    # if cv2.waitKey(0) & 0xFF == 27:
    if cv2.waitKey(1) & 0xFF == 27:
        break

cap.release()
cv2.destroyAllWindows()