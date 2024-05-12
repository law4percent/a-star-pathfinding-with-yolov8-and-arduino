import cv2
from ultralytics import YOLO
import cvzone
import numpy as np
import pandas as pd
# import serial

def VideoFrame(event, x, y, flags, param):
    if event == cv2.EVENT_MOUSEMOVE:
        point = [x, y]
        print(point)

def Split_Class_List(file_path):
    myFile = open(file_path, "r")
    data = myFile.read()
    class_list = data.split("\n")
    myFile.close()
    
    return class_list

def main():
    cv2.namedWindow("Video Frame")
    cv2.setMouseCallback("Video Frame", VideoFrame)

    VIDEO_SOURCE_PATH = "inference/Videos/SAMPLE_VIDEO.mp4"
    # VIDEO_SOURCE_PATH = "http://192.168.1.2:8080/video"
    yolov8_weights = "weights/yolov8n.pt"
    COCO_FILE_PATH = "utils/coco.txt"

    model = YOLO(yolov8_weights, "v8")
    cap = cv2.VideoCapture(VIDEO_SOURCE_PATH)
    # cap1 = cv2.VideoCapture(VIDEO_SOURCE_PATH)
    class_list = Split_Class_List(COCO_FILE_PATH) 

    count = 0
    frame_width = 1280 # 1020
    frame_height = 720 # 500

    if not cap.isOpened():
        print("Cannot open camera")
        exit()
    #                   TL           BL          BR          TR
    mid_path_area = [(469, 318), (469, 492), (816, 492), (816, 318)]
    left_path_area = [(164, 318), (164, 492), (463, 492), (463, 318)]
    right_path_area = [(821, 318), (821, 492), (1080, 492), (1080, 318)]
    bottom_path_area = [(469, 496), (469, 698), (816, 698), (816, 496)]
    top_path_area = [(469, 150), (469, 313), (816, 313), (816, 150)]

    while True:
        success, frame = cap.read()
        # success1, frame1 = cap1.read()
        # frame1 = cv2.resize(frame1, (frame_width, frame_height))

        if not success:
            cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            continue

        count += 1
        if count % 3 != 0:
            continue

        frame = cv2.resize(frame, (frame_width, frame_height))
        predict_results = model.predict(source=[frame], conf=0.45, save=False)
        pred_res = predict_results[0].boxes.data
        px = pd.DataFrame(pred_res).astype("float")

        mid_list = []
        left_list = []
        right_list = []
        bottom_list = []
        top_list = []

        for index, row in px.iterrows():
            x1 = int(row[0])
            y1 = int(row[1])
            x2 = int(row[2])
            y2 = int(row[3])

            confidence = round(row[4], 5)
            detected_class_index = int(row[5])
            class_ID_name = class_list[detected_class_index]

            cx = int(x1 + x2) // 2
            cy = int(y1 + y2) // 2
            w, h = x2 - x1, y2 - y1

            mid_path_result = cv2.pointPolygonTest(np.array(mid_path_area, np.int32), ((cx, cy)), False)
            left_path_result = cv2.pointPolygonTest(np.array(left_path_area, np.int32), ((cx, cy)), False)
            right_path_result = cv2.pointPolygonTest(np.array(right_path_area, np.int32), ((cx, cy)), False)
            bottom_path_result = cv2.pointPolygonTest(np.array(bottom_path_area, np.int32), ((cx, cy)), False)
            top_path_result = cv2.pointPolygonTest(np.array(top_path_area, np.int32), ((cx, cy)), False)

            if mid_path_result >= 0:
                cvzone.cornerRect(frame, (x1, y1, w, h), 3, 2)
                cv2.circle(frame, (cx, cy), 4, (255, 0, 0), -1)
                cvzone.putTextRect(frame, f"{class_ID_name} {confidence}%", (x1, y1), 1, 1)
                mid_list.append(cx)

            if left_path_result >= 0:
                cvzone.cornerRect(frame, (x1, y1, w, h), 3, 2)
                cv2.circle(frame, (cx, cy), 4, (255, 0, 255), -1)
                cvzone.putTextRect(frame, f"{class_ID_name} {confidence}%", (x1, y1), 1, 1)
                left_list.append(cx)
            
            if right_path_result >= 0:
                cvzone.cornerRect(frame, (x1, y1, w, h), 3, 2)
                cv2.circle(frame, (cx, cy), 4, (255, 0, 255), -1)
                cvzone.putTextRect(frame, f"{class_ID_name} {confidence}%", (x1, y1), 1, 1)
                right_list.append(cx)

            if bottom_path_result >= 0:
                cvzone.cornerRect(frame, (x1, y1, w, h), 3, 2)
                cv2.circle(frame, (cx, cy), 4, (255, 0, 255), -1)
                cvzone.putTextRect(frame, f"{class_ID_name} {confidence}%", (x1, y1), 1, 1)
                bottom_list.append(cx)
            
            if top_path_result >= 0:
                cvzone.cornerRect(frame, (x1, y1, w, h), 3, 2)
                cv2.circle(frame, (cx, cy), 4, (255, 0, 255), -1)
                cvzone.putTextRect(frame, f"{class_ID_name} {confidence}%", (x1, y1), 1, 1)
                top_list.append(cx)

        numberOfClassDetected_midPath = len(mid_list)
        numberOfClassDetected_leftPath = len(left_list)
        numberOfClassDetected_rightPath = len(right_list)
        numberOfClassDetected_bottomPath = len(bottom_list)
        numberOfClassDetected_topPath = len(top_list)

        cv2.polylines(frame, [np.array(mid_path_area, np.int32)], True, (0, 0, 255), 2)
        cv2.polylines(frame, [np.array(left_path_area, np.int32)], True, (0, 255, 0), 2)
        cv2.polylines(frame, [np.array(right_path_area, np.int32)], True, (0, 255, 0), 2)
        cv2.polylines(frame, [np.array(bottom_path_area, np.int32)], True, (0, 255, 0), 2)
        cv2.polylines(frame, [np.array(top_path_area, np.int32)], True, (0, 255, 0), 2)

        cvzone.putTextRect(frame, f"Middle Zone: {numberOfClassDetected_midPath} || Left Zone: {numberOfClassDetected_leftPath}", pos=(50, 60), scale=1, thickness=2, colorR=(100, 100, 0))
        cvzone.putTextRect(frame, f"Right Zone: {numberOfClassDetected_rightPath}  || Bottom Zone: {numberOfClassDetected_bottomPath}", pos=(50, 95), scale=1, thickness=2, colorR=(255, 0, 0))
        cvzone.putTextRect(frame, f"Top Zone: {numberOfClassDetected_topPath}", pos=(50, 130), scale=1, thickness=2, colorR=(100, 0, 100))

        cvzone.putTextRect(frame, f"Table A", pos=(347, 229), scale=1, thickness=2, colorT=(0, 0, 255))
        cvzone.putTextRect(frame, f"Table B", pos=(895, 229), scale=1, thickness=2, colorT=(0, 0, 255))
        cvzone.putTextRect(frame, f"Table C", pos=(347, 497), scale=1, thickness=2, colorT=(0, 0, 255))
        cvzone.putTextRect(frame, f"Table D", pos=(895, 497), scale=1, thickness=2, colorT=(0, 0, 255))

        cv2.imshow("Video Frame", frame)
        # cv2.imshow("Video Frame1", frame1)

        # if cv2.waitKey(1) == ord("q"):
        if cv2.waitKey(0) & 0xFF == 27: # ESC
            break

    cap.release()
    # cap1.release()
    cv2.destroyAllWindows()
    

if __name__ == "__main__":
    main()