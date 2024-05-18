import cv2
from ultralytics import YOLO
import cvzone
import numpy as np
import pandas as pd
import serial
import time

import calculate_distance as cd
import canteen_areas as ca
from canteen_areas import Area, _center_X_, _top_left_corner_

# Show_Text_TrgtAreas = False
Show_Text_NumOfClass = True
Show_Zones = True
Show_BoundingBox_ClsID = False
ARDUINO = False
Show_Area_Index = True

if ARDUINO:
    serial_port = 'COM3'
    baud_rate = 9600
    global arduino
    arduino = serial.Serial(serial_port, baud_rate)

def sendCommandToArduino(command):
    global arduino
    arduino.write(command.encode('utf-8'))
    print("Sent command to Arduino:", command)

def decodeReceivedDataCommand():
    global arduino
    received_command = arduino.readline().decode('utf-8').rstrip()
    print(f"Received: {received_command}")
    return received_command

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

def boundingBox_ClsID_display(Frame, Rec_pos, Color, Text, Text_pos):
    cvzone.cornerRect(Frame, Rec_pos, l=0, t=2, rt=2, colorR=Color, colorC=Color)
    cvzone.putTextRect(Frame, Text, Text_pos, 1, 1, colorR=Color)
    
def PolygonTest(Area, XY):
    return cv2.pointPolygonTest(np.array(Area, np.int32), XY, False)

def main():
    frame_width = 1280 # 1020
    frame_height = 720 # 500

    transform_frame_name = "Birds Eye View"
    window_frame_name = "Normal Video"
    cv2.namedWindow(transform_frame_name)
    cv2.setMouseCallback(transform_frame_name, VideoFrame)

    VIDEO_SOURCE_PATH = "inference/Videos/SAMPLE_VIDEO.mp4"
    yolov8_weights = "weights/best2.pt"
    COCO_FILE_PATH = "utils/coco1.txt"

    model = YOLO(yolov8_weights, "v8")
    cap = cv2.VideoCapture(VIDEO_SOURCE_PATH)
    class_list = Split_Class_List(COCO_FILE_PATH) 

    count = 0

    if not cap.isOpened():
        print("Cannot open camera")
        exit()

    while True:
        success, frame = cap.read()

        frame = cv2.resize(frame, (frame_width, frame_height))
        pnts1 = np.float32([ca.entire_area[0], ca.entire_area[1], ca.entire_area[3], ca.entire_area[2]])
        pnts2 = np.float32([[0,0], [0,frame_height], [frame_width,0], [frame_width,frame_height]])

        matrix = cv2.getPerspectiveTransform(pnts1, pnts2)
        transform_frame = cv2.warpPerspective(frame, matrix, (frame_width, frame_height))

        if not success:
            cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            continue
        
        if ARDUINO:
            if arduino.in_waiting > 0:
                requestIDandCommand = decodeReceivedDataCommand() # to receive TABLE_A_1 ||TABLE_A_0
                reqFlag = True
            else:
                pass
        
            
        count += 1
        if count % 5 != 0:
            continue

        
        detect_results = model.predict(source=[transform_frame], conf=0.45, save=False)
        pred_res = detect_results[0].boxes.data
        px = pd.DataFrame(pred_res).astype("float")

        PL_00 = []
        PL_01 = []
        PL_02 = []
        PL_03 = []
        PL_04 = []
        PL_05 = []
        PL_06 = []
        PL_07 = []

        PL_10 = []
        PL_11 = []
        PL_12 = []
        PL_13 = []
        PL_14 = []
        PL_15 = []
        PL_16 = []
        PL_17 = []

        PL_20 = []
        PL_21 = []
        PL_22 = []
        PL_23 = []
        PL_24 = []
        PL_25 = []
        PL_26 = []
        PL_27 = []

        PL_30 = []
        PL_31 = []
        PL_32 = []
        PL_33 = []
        PL_34 = []
        PL_35 = []
        PL_36 = []
        PL_37 = []

        PL_40 = []
        PL_41 = []
        PL_42 = []
        PL_43 = []
        PL_44 = []
        PL_45 = []
        PL_46 = []
        PL_47 = []

        PL_50 = []
        PL_51 = []
        PL_52 = []
        PL_53 = []
        PL_54 = []
        PL_55 = []
        PL_56 = []
        PL_57 = []

        path_lists = (
            PL_00, PL_01, PL_02, PL_03, PL_04, PL_05, PL_06, PL_07,
            PL_10, PL_11, PL_12, PL_13, PL_14, PL_15, PL_16, PL_17,
            PL_20, PL_21, PL_22, PL_23, PL_24, PL_25, PL_26, PL_27,
            PL_30, PL_31, PL_32, PL_33, PL_34, PL_35, PL_36, PL_37,
            PL_40, PL_41, PL_42, PL_43, PL_44, PL_45, PL_46, PL_47,
            PL_50, PL_51, PL_52, PL_53, PL_54, PL_55, PL_56, PL_57,
        )

        binary_map = [
                        1, 1, 1, 1, 1, 1, 1, 1,
                        1, 0, 0, 1, 1, 0, 0, 1,
                        1, 1, 1, 1, 1, 1, 1, 1,
                        1, 1, 1, 1, 1, 1, 1, 1,
                        1, 0, 0, 1, 1, 0, 0, 1,
                        1, 1, 1, 1, 1, 1, 1, 1,
                    ]
        
        sum_of_cls = 0

        for index, row in px.iterrows():
            x1 = int(row[0])
            y1 = int(row[1])
            x2 = int(row[2])
            y2 = int(row[3])

            confidence = round(row[4], 5)
            detected_class_index = int(row[5])
            class_ID_name = class_list[detected_class_index]

            cls_cx = int(x1 + x2) // 2
            cls_cy = int(y1 + y2) // 2
            cls_center = (cls_cx, cls_cy)
            w, h = x2 - x1, y2 - y1

            rec_pos = (x1, y1, w, h)
            text_pos = (x1, y1)
            clsID_and_Conf = f"{class_ID_name} {confidence}%"

            for index in range(len(Area)):
                if PolygonTest(Area=Area[index], XY=cls_center) >= 0:
                    if Show_BoundingBox_ClsID:
                        boundingBox_ClsID_display(Frame=transform_frame, Rec_pos=rec_pos, Color=(0, 0 ,255), Text=clsID_and_Conf, Text_pos=text_pos)
                    cv2.circle(transform_frame, cls_center, 4, (0, 0 ,255), -1)
                    list_to_append = path_lists[index]
                    list_to_append.append(cls_cx)
                    if binary_map[index] != 0:
                        binary_map[index] = 0 if len(list_to_append) > 0 else 1
                    sum_of_cls += len(list_to_append)

        
        print("======= Binary Map =======\n")
        for index in range(len(Area)):
            print(f"{binary_map[index]} ", end="")
            if (index+1) % 8 == 0:
                print("")
        print("\n======= End Map =======")

        if Show_Zones:
            warningColor = (0, 0, 255)
            defaultColor = (0, 200, 0)
            fontFace = cv2.FONT_HERSHEY_SIMPLEX
            fontScale = 0.5
            textThickness = 2

            cv2.polylines(frame, [np.array(ca.entire_area, np.int32)], True, warningColor, 2)
            for index in range(len(Area)):
                color = defaultColor if binary_map[index] > 0 else warningColor
                cv2.polylines(transform_frame, [np.array(Area[index], np.int32)], True, color, 2)
                if Show_Text_NumOfClass:
                    cv2.putText(transform_frame, f"{0 if len(path_lists[index]) > 0 else 1}", _center_X_[index], fontFace=fontFace, fontScale=fontScale, color=(255, 255, 255), thickness=textThickness)
                    cv2.putText(transform_frame, f"Total Class: {sum_of_cls}", (1100, 35), fontFace=fontFace, fontScale=fontScale, color=(255, 255, 255), thickness=textThickness)
                if Show_Area_Index:
                    cv2.putText(transform_frame, f"{index}", _top_left_corner_[index], fontFace=fontFace, fontScale=fontScale, color=(255, 255, 255), thickness=textThickness)

        cv2.imshow(transform_frame_name, transform_frame)
        # cv2.imshow(window_frame_name, frame)

        if cv2.waitKey(0) & 0xFF == 27: # ESC
            break


    cap.release()
    cv2.destroyAllWindows()
    if ARDUINO:
        arduino.close()
    
if __name__ == "__main__":
    main()