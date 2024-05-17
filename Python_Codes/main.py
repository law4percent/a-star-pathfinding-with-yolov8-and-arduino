import cv2
from ultralytics import YOLO
import cvzone
import numpy as np
import pandas as pd
import serial
import time

import calculate_distance as CD
from canteen_areas import *

Show_Text_TrgtAreas = True
Show_Text_NumOfClass = True
Show_Zones = True
Show_BoundingBox_ClsID = False
ARDUINO = False

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
    window_frame_name = "Video Frame"

    cv2.namedWindow(window_frame_name)
    cv2.setMouseCallback(window_frame_name, VideoFrame)

    VIDEO_SOURCE_PATH = "inference/Videos/SAMPLE_VIDEO.mp4"
    yolov8_weights = "weights/best2.pt"
    COCO_FILE_PATH = "utils/coco1.txt"

    model = YOLO(yolov8_weights, "v8")
    cap = cv2.VideoCapture(VIDEO_SOURCE_PATH)
    class_list = Split_Class_List(COCO_FILE_PATH) 

    count = 0
    frame_width = 1280 # 1020
    frame_height = 720 # 500

    if not cap.isOpened():
        print("Cannot open camera")
        exit()

    M_LRTBcolor = (237, 227, 90)
    TB_LRcolor = (252, 3, 161)
    mcolor = (0, 200, 250)

    area_color_list = [
                (Tleft_path_area, TB_LRcolor),
                (Tmid_path_area, M_LRTBcolor),
                (Tright_path_area, TB_LRcolor),

                (Mleft_path_area, M_LRTBcolor),
                (mid_path_area, mcolor),
                (Mright_path_area, M_LRTBcolor),

                (Bleft_path_area, TB_LRcolor),
                (Bmid_path_area, M_LRTBcolor),
                (Bright_path_area, TB_LRcolor)
            ]

    while True:
        success, frame = cap.read()

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

        frame = cv2.resize(frame, (frame_width, frame_height))
        detect_results = model.predict(source=[frame], conf=0.45, save=False)
        pred_res = detect_results[0].boxes.data
        px = pd.DataFrame(pred_res).astype("float")

        
        Tleft_list = []
        Tmid_list = []
        Tright_list = []

        mid_list = []
        Mleft_list = []
        Mright_list = []

        Bleft_list = []
        Bmid_list = []
        Bright_list = []

        listOfPathList = (
            Tleft_list,
            Tmid_list,
            Tright_list,

            Mleft_list,
            mid_list,
            Mright_list,

            Bleft_list,
            Bmid_list,
            Bright_list
        )

        for index, row in px.iterrows():
            x1 = int(row[0])
            y1 = int(row[1])
            x2 = int(row[2])
            y2 = int(row[3])

            confidence = round(row[4], 5)
            detected_class_index = int(row[5])
            class_ID_name = class_list[detected_class_index]

            cls_x = int(x1 + x2) // 2
            cls_y = int(y1 + y2) // 2
            cls_xy = (cls_x, cls_y)
            w, h = x2 - x1, y2 - y1

            rec_pos = (x1, y1, w, h)
            text_pos = (x1, y1)
            clsID_and_Conf = f"{class_ID_name} {confidence}%"

            for area, color in area_color_list:
                if PolygonTest(Area=area, XY=cls_xy) >= 0:
                    if Show_BoundingBox_ClsID:
                        boundingBox_ClsID_display(Frame=frame, Rec_pos=rec_pos, Color=color, Text=clsID_and_Conf, Text_pos=text_pos)
                    cv2.circle(frame, cls_xy, 4, color, -1)
                    list_to_append = listOfPathList[area_color_list.index((area, color))]
                    list_to_append.append(cls_x)


        numClsFnd_mPath = len(mid_list)
        numClsFnd_MlPath = len(Mleft_list)
        numClsFnd_MrPath = len(Mright_list)

        numClsFnd_TlPath = len(Tleft_list)
        numClsFnd_TmPath = len(Tmid_list)
        numClsFnd_TrPath = len(Tright_list)

        numClsFnd_BlPath = len(Bleft_list)
        numClsFnd_BmPath = len(Bmid_list)
        numClsFnd_BrPath = len(Bright_list)

        totalNumOfClsDetected = [numClsFnd_mPath, numClsFnd_MlPath, numClsFnd_MrPath,
                                numClsFnd_BmPath, numClsFnd_TmPath, numClsFnd_TlPath,
                                numClsFnd_BlPath, numClsFnd_TrPath, numClsFnd_BrPath]


        if Show_Text_NumOfClass:
            colorR = (100, 100, 0)
            texts = [
                        f"Middle Zone: {numClsFnd_mPath}",
                        f"M-Left Zone: {numClsFnd_MlPath} || M-Right Zone: {numClsFnd_MrPath}",
                        f"T-Left Zone: {numClsFnd_TlPath} || B-Left Zone: {numClsFnd_BlPath}",
                        f"T-Mid Zone: {numClsFnd_TmPath} || B-Mid Zone: {numClsFnd_BmPath}",
                        f"T-Right Zone: {numClsFnd_TrPath} || B-Right Zone: {numClsFnd_BrPath}"
                    ]
            for i, txt in enumerate(texts):
                cvzone.putTextRect(frame, txt, pos=(30, 30 + 34 * i), scale=1, thickness=2, colorR=colorR)
            cvzone.putTextRect(frame, f"Total Class Found: {sum(totalNumOfClsDetected)}", pos=(200, 30), scale=1, thickness=2, colorR=None)

        if Show_Zones:
            Redcolor = (0, 0, 255)

            paths = [
                        (Tleft_path_area, numClsFnd_TlPath, TB_LRcolor, TB_LRcolor),
                        (Tmid_path_area, numClsFnd_TmPath, M_LRTBcolor, M_LRTBcolor),
                        (Tright_path_area, numClsFnd_TrPath, TB_LRcolor, TB_LRcolor),

                        (Mleft_path_area, numClsFnd_MlPath, M_LRTBcolor, M_LRTBcolor),
                        (mid_path_area, numClsFnd_mPath, mcolor, M_LRTBcolor),
                        (Mright_path_area, numClsFnd_MrPath, M_LRTBcolor, M_LRTBcolor),

                        (Bleft_path_area, numClsFnd_BlPath, TB_LRcolor, TB_LRcolor),
                        (Bmid_path_area, numClsFnd_BmPath, M_LRTBcolor, M_LRTBcolor),
                        (Bright_path_area, numClsFnd_BrPath, TB_LRcolor, TB_LRcolor),
                    ]

            for path_area, numClsFnd, default_color, secondary_color in paths:
                color = Redcolor if numClsFnd > 0 else default_color
                cv2.polylines(frame, [np.array(path_area, np.int32)], True, color, 2)

        if Show_Text_TrgtAreas:
            fontFace = cv2.FONT_HERSHEY_SIMPLEX
            fontScale = 0.5
            color = (255, 255, 255)
            textThickness = 2
            circleThickness = -1
            circleColor = (0, 255, 0)
            radius = 5

            TableLocation = (tableA_pos, tableB_pos, tableC_pos, tableD_pos)
            AreasPoint = (pointTL_pos, pointTM_pos, pointTR_pos, pointML_pos, pointM_pos, pointMR_pos, pointBL_pos, pointBM_pos, pointBR_pos)
            points = [
                        ('TL', pointTL_pos),
                        ('TM', pointTM_pos),
                        ('TR', pointTR_pos),
                        ('ML', pointML_pos),
                        ('M', pointM_pos),
                        ('MR', pointMR_pos),
                        ('BL', pointBL_pos),
                        ('BM', pointBM_pos),
                        ('BR', pointBR_pos)
                    ]

            for index in range(len(TableLocation)):
                cv2.putText(frame, f"Table A {TableLocation[index]}", TableLocation[index], fontFace=fontFace, fontScale=fontScale, color=color, thickness=textThickness)

            for index in range(len(AreasPoint)):
                cv2.circle(frame, AreasPoint[index], radius=radius, color=circleColor, thickness=circleThickness)

            for label, pos in points:
                cv2.putText(frame, f"{label} {pos}", (pos[0] - 35, pos[1] - 15), fontFace=fontFace, fontScale=fontScale, color=color, thickness=textThickness)
            
            # cv2.putText(frame, f"Target Area: {target_location}", org=(1050, 30), fontFace=fontFace, fontScale=fontScale, color=color, thickness=textThickness)

        cv2.imshow(window_frame_name, frame)

        if cv2.waitKey(0) & 0xFF == 27: # ESC
            break


    cap.release()
    cv2.destroyAllWindows()
    if ARDUINO:
        arduino.close()
    
if __name__ == "__main__":
    main()