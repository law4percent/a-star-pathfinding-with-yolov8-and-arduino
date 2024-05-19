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

def videoFrame(event, x, y, flags, param):
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

def ConvertToMatrix_Array_of_Array(_array):
    rows = 6
    columns = 8
    matrix = [[0] * columns for _ in range(rows)]
    
    _row = 0
    _column = 0

    for index in range(len(_array)):
        matrix[_row][_column] = _array[index]
        _column += 1
        if _column == columns:
            _column = 0
            _row += 1

    return matrix

def ConvertToListOfTuple(path):
    path_list= []
    for i in range(len(path)):
        path_list.append(tuple(path[i]))
    return path_list

def ConvertToBirdEyeView(frame, matrix, HW):
    return cv2.warpPerspective(frame, matrix, HW)

def getTheTargetArea(targetTable_listOfnear_areaXY:list, numberOfCls_eachArea:list, Robot_Current_Location:tuple) -> tuple:
    # ELIMINATE Areas that have more than 2 cls
    max_cls = 3
    get_remain_areas = []
    count = 0

    for index in range(len(numberOfCls_eachArea)):
        if numberOfCls_eachArea[index] >= 1: #max_cls:
            pass
        else:
            count += 1
            get_remain_areas.append(targetTable_listOfnear_areaXY[index])

        # Find the shortest area
        if count == 3:
            min_distance = float('inf')
            closest_area = None
            print("Same all values")
            for area in get_remain_areas:
                distance = cd.calculateDistance(pointA=Robot_Current_Location, pointB=area)
                if distance < min_distance:
                    min_distance = distance
                    closest_area = area
            return closest_area


    # Find the shortest area
    if len(get_remain_areas) > 1:
        min_distance = float('inf')
        closest_area = None
        for area in get_remain_areas:
            distance = cd.calculateDistance(pointA=Robot_Current_Location, pointB=area)
            if distance < min_distance:
                min_distance = distance
                closest_area = area
        return closest_area
    
    elif len(get_remain_areas) == 1:
        return get_remain_areas[0]

def main():
    frame_width = 1280 # 1020
    frame_height = 720 # 500

    transform_frame_name = "Bird's Eye View"
    window_frame_name = "Normal Video"
    cv2.namedWindow(transform_frame_name)
    cv2.setMouseCallback(transform_frame_name, videoFrame)

    VIDEO_SOURCE_PATH = "inference/Videos/SAMPLE_VIDEO.mp4"
    yolov8_weights = "weights/best2.pt"
    COCO_FILE_PATH = "utils/coco1.txt"

    model = YOLO(yolov8_weights, "v8")
    cap = cv2.VideoCapture(VIDEO_SOURCE_PATH)
    class_list = Split_Class_List(COCO_FILE_PATH) 

    count = 0

    pnts1 = np.float32([ca.entire_area[0], ca.entire_area[1], ca.entire_area[3], ca.entire_area[2]])
    pnts2 = np.float32([[0,0], [0,frame_height], [frame_width,0], [frame_width,frame_height]])
    matrix = cv2.getPerspectiveTransform(pnts1, pnts2)

    list_requestTable_queu = []
    currentRequestingTable = ""
    go_flag = False
    createPath = False
    shortestPath_x = 0
    shortestPath_y = 0
    Table_A_Flag = False
    Table_B_Flag = False
    Table_C_Flag = False
    Table_D_Flag = False

    if not cap.isOpened():
        print("Cannot open camera")
        exit()

    while True:
        success, frame = cap.read()
        frame = cv2.resize(frame, (frame_width, frame_height))
        transform_frame = ConvertToBirdEyeView(frame, matrix, (frame_width, frame_height))

        if not success:
            cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            continue
        
        if ARDUINO:
            if arduino.in_waiting > 0:
                requestNameandCommand = decodeReceivedDataCommand() # to receive A_1 || _A_0
                if requestNameandCommand[0] == ca.Tables_NearAreas[0]["table_name"]:
                    if requestNameandCommand[2] == "1":
                        Table_A_Flag = True
          
        count += 1
        if count % 5 != 0:
            continue
        
        pred_result = model.predict(source=[transform_frame], conf=0.45, save=False)
        px = pd.DataFrame(pred_result[0].boxes.data).astype("float")

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

        binary_map = [# 0  1  2  3  4  5  6  7
                        1, 1, 1, 1, 1, 1, 1, 1, # 0
                        1, 0, 0, 1, 1, 0, 0, 1, # 1
                        1, 0, 0, 1, 1, 0, 0, 1, # 2
                        1, 1, 1, 1, 1, 1, 1, 1, # 3
                        1, 0, 0, 1, 1, 0, 0, 1, # 4
                        1, 1, 1, 1, 1, 1, 1, 1, # 5
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

            cls_center_x = int(x1 + x2) // 2
            cls_center_y = int(y1 + y2) // 2
            cls_center_pnt = (cls_center_x, cls_center_y)
            w, h = x2 - x1, y2 - y1

            rec_pos = (x1, y1, w, h)
            text_pos = (x1, y1)
            clsID_and_Conf = f"{class_ID_name} {confidence}%"

            for index in range(len(Area)):
                if PolygonTest(Area=Area[index], XY=cls_center_pnt) >= 0:
                    centerPnt_BndBoxColor = (0, 0, 255)
                    if Show_BoundingBox_ClsID:
                        boundingBox_ClsID_display(Frame=transform_frame, Rec_pos=rec_pos, Color=centerPnt_BndBoxColor, Text=clsID_and_Conf, Text_pos=text_pos)
                    cv2.circle(transform_frame, cls_center_pnt, 5, centerPnt_BndBoxColor, -1) #158, 245, 255
                    list_to_append = path_lists[index]
                    list_to_append.append(cls_center_x)
                    binary_map[index] = 0 if len(list_to_append) > 0 else 1
                    sum_of_cls += len(list_to_append)

        print(f"A Top: {len(PL_02)} Right: {len(PL_23)} Bottom: {len(PL_32)} Left: {len(PL_20)}")
        print(f"B Top: {len(PL_06)} Right: {len(PL_27)} Bottom: {len(PL_36)} Left: {len(PL_24)}")
        print(f"C Top: {len(PL_32)} Right: {len(PL_43)} Bottom: {len(PL_52)} Left: {len(PL_40)}")
        print(f"D Top: {len(PL_36)} Right: {len(PL_47)} Bottom: {len(PL_56)} Left: {len(PL_44)}")

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
                    cv2.putText(transform_frame, f"{binary_map[index]}", _center_X_[index], fontFace=fontFace, fontScale=fontScale, color=(255, 255, 255), thickness=textThickness)
                    cv2.putText(transform_frame, f"Total Class: {sum_of_cls}", (1100, 35), fontFace=fontFace, fontScale=fontScale, color=(255, 255, 255), thickness=textThickness)
                if Show_Area_Index:
                    cv2.putText(transform_frame, f"{index}", _top_left_corner_[index], fontFace=fontFace, fontScale=fontScale, color=(255, 255, 255), thickness=textThickness)

        print("======= Binary Map =======\n")
        for index in range(len(Area)):
            print(f"{binary_map[index]} ", end="")
            if (index+1) % 8 == 0:
                print()
        print("\n======= Binary Map End =======\n")

        robot_default_location = [0, 5]
        Robot_Current_Location = [0, 0]
        target_area = None

        if Table_A_Flag:
            TableA_listOf_areaXY = ca.TableA_nearArea
            numberOfCls_eachArea = [len(PL_02), len(PL_23), len(PL_32), len(PL_20)]
            target_area = getTheTargetArea(TableA_listOf_areaXY, numberOfCls_eachArea, Robot_Current_Location)
            Table_A_Flag = False 

        elif Table_B_Flag:
            TableB_listOf_areaXY = ca.TableB_nearArea
            numberOfCls_eachArea = [len(PL_06), len(PL_27), len(PL_36), len(PL_24)]
            target_area = getTheTargetArea(TableB_listOf_areaXY, numberOfCls_eachArea, Robot_Current_Location)
            Table_B_Flag = False

        elif Table_C_Flag:
            TableC_listOf_areaXY = ca.TableC_nearArea
            numberOfCls_eachArea = [len(PL_32), len(PL_43), len(PL_52), len(PL_40)]
            target_area = getTheTargetArea(TableC_listOf_areaXY, numberOfCls_eachArea, Robot_Current_Location)
            Table_C_Flag = False

        elif Table_D_Flag == False:
            TableD_listOf_areaXY = ca.TableD_nearArea
            numberOfCls_eachArea = [len(PL_36), len(PL_47), len(PL_56), len(PL_44)]
            target_area = getTheTargetArea(TableD_listOf_areaXY, numberOfCls_eachArea, Robot_Current_Location)
            Table_D_Flag = False

        if target_area != None:
            _matrix = ConvertToMatrix_Array_of_Array(binary_map)
            shortest_pathway = cd.CreatePath(Robot_Current_Location, target_area, _matrix)
            cv2.putText(transform_frame, f"Target Area: {target_area}", (50, 35), fontFace=fontFace, fontScale=fontScale, color=(255, 255, 255), thickness=textThickness)
            cv2.putText(transform_frame, f"Path way: {ConvertToListOfTuple(shortest_pathway)}", (50, 720-35), fontFace=fontFace, fontScale=fontScale, color=(255, 255, 255), thickness=textThickness)

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