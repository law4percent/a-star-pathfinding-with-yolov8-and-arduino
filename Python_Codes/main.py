import cv2
from ultralytics import YOLO
import cvzone
import numpy as np
import pandas as pd
import serial
import time

import calculate_distance as cd
import canteen_areas as ca
from canteen_areas import Area, _center_X_, _bottom_left_corner_

Birds_Eye_View = True
ShowOnFrame_EntireArea_Zone = True
ShowOnFrame_Zones = True
ShowOnFrame_RobotPathZones = True
ShowOnFrame_ObstacledZones = True
ShowOnFrame_NumOfClass = True
ShowOnFrame_BoundingBoxAndClsID = False
ShowOnFrame_IndexNumOfEveryArea = False
ShowOnFrame_Binary = False
ShowTransformFrame = True
ShowNormalFrame = False
ARDUINO = False
DEBUG_CMD = True
DEBUG_FRAME = True
MouseCallBack = False

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

def ConvertToMatrixOfArray(_array, max_row, max_col, display_data=True):
    rows = max_row
    columns = max_col
    matrix = [[0] * columns for _ in range(rows)]
    
    _row = 0
    _column = 0

    for index in range(len(_array)):
        matrix[_row][_column] = _array[index]
        _column += 1
        if _column == columns:
            _column = 0
            _row += 1

    if display_data:
        print("\n======== Matrix ========\n")
        for row in matrix:
            print(row)
        print("\n======== End Matrix ========\n")
    return matrix

def ConvertToListOfTuple(path):
    path_list= []
    for i in range(len(path)):
        path_list.append(tuple(path[i]))
    return path_list

def ConvertToBirdEyeView(frame, matrix, HW):
    return cv2.warpPerspective(frame, matrix, HW)

def getTheTargetArea(targetTable_listOfnear_areaXY:list, numberOfCls_eachArea:list, startArea:tuple, max_cls, display_data=True) -> tuple:
    get_remain_areas = []
    count = 0
    # ELIMINATE Areas that have more than max_cls
    for index in range(len(numberOfCls_eachArea)):
        if numberOfCls_eachArea[index] < max_cls:
            count += 1
            get_remain_areas.append(targetTable_listOfnear_areaXY[index])

        # Find the shortest area
        if count == 3:
            if display_data:
                print("Areas are the same values of class.")
            min_distance = float('inf')
            closest_area = None

            for targetArea in get_remain_areas:
                distance = cd.calculateDistance(pointA=startArea, pointB=targetArea)
                if distance < min_distance:
                    min_distance = distance
                    closest_area = targetArea
            return closest_area

    # Find the shortest area
    if len(get_remain_areas) > 1:
        min_distance = float('inf')
        closest_area = None
        for targetArea in get_remain_areas:
            distance = cd.calculateDistance(pointA=startArea, pointB=targetArea)
            if distance < min_distance:
                min_distance = distance
                closest_area = targetArea
        return closest_area
    
    elif len(get_remain_areas) == 1:
        return get_remain_areas[0]

def navigateRobotLocation(array1D, row, col, keyword, display_output=True):
    matrixOfnumOfCls = ConvertToMatrixOfArray(array1D, row, col, False)
    countRow = 0
    countCol = 0
    for Row in matrixOfnumOfCls:
        for Col in Row:
            if Col == keyword:
                X = countCol
                Y = countRow
                location = (X, Y)
                if display_output:
                    print(f"Robot Loction: {location}")
                return location
            countCol += 1
        countCol = 0
        countRow += 1

def main():
    transform_frame_name = "Bird's Eye View"
    window_frame_name = "Normal Video"
    
    if Birds_Eye_View:
        if MouseCallBack:
            cv2.namedWindow(transform_frame_name)
            cv2.setMouseCallback(transform_frame_name, videoFrame)
    else:
        if MouseCallBack:
            cv2.namedWindow(window_frame_name)
            cv2.setMouseCallback(window_frame_name, videoFrame)

    VIDEO_SOURCE_PATH = "inference/Videos/Sample_Video.mp4"
    yolov8_weights = "weights/OSS_Weight.pt"
    COCO_FILE_PATH = "utils/coco.names"

    model = YOLO(yolov8_weights, "v8")
    cap = cv2.VideoCapture(VIDEO_SOURCE_PATH)
    class_list = Split_Class_List(COCO_FILE_PATH) 

    count = 0
    frame_width = 1280 # 1020
    frame_height = 720 # 500
    max_cls_on_area = 1
    frame_column = 8
    frame_row = 6
    pnts1 = np.float32([ca.entire_area[0], ca.entire_area[1], ca.entire_area[3], ca.entire_area[2]])
    pnts2 = np.float32([[0,0], [0,frame_height], [frame_width,0], [frame_width, frame_height]])
    matrix = cv2.getPerspectiveTransform(pnts1, pnts2)
    
    if not cap.isOpened():
        print("Cannot open camera")
        exit()

    Table_A_Flag = False
    Table_B_Flag = False
    Table_C_Flag = False
    Table_D_Flag = True

    startTime = 0
    interval_reset = 10

    while True:
        success, frame = cap.read()

        if ARDUINO:
            if arduino.in_waiting > 0:
                requestNameandCommand = decodeReceivedDataCommand() # to receive A1 || A0
                if requestNameandCommand == "CP":
                    pass
                else:
                    if requestNameandCommand[0] == ca.Table_Names[0]:
                        if requestNameandCommand[1] == "1":
                            Table_A_Flag = True
                            startTime = time.time()
                        else:
                            Table_A_Flag = False
                            startTime = 0
                    
                    elif requestNameandCommand[0] == ca.Table_Names[1]:
                        if requestNameandCommand[1] == "1":
                            Table_B_Flag = True
                            startTime = time.time()
                        else:
                            Table_B_Flag = False
                            startTime = 0

                    elif requestNameandCommand[0] == ca.Table_Names[2]:
                        if requestNameandCommand[1] == "1":
                            Table_C_Flag = True
                            startTime = time.time()
                        else:
                            Table_C_Flag = False
                            startTime = 0

                    elif requestNameandCommand[0] == ca.Table_Names[3]:
                        if requestNameandCommand[1] == "1":
                            Table_D_Flag = True
                            startTime = time.time()
                        else:
                            Table_D_Flag = False
                            startTime = 0

        if not success:
            cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            continue
        
        count += 1
        if count % 15 != 0:
            continue
        
        frame = cv2.resize(frame, (frame_width, frame_height))
        transform_frame = ConvertToBirdEyeView(frame, matrix, (frame_width, frame_height))
        bird_eye_Pred_result = model.predict(source=[transform_frame], conf=0.45, save=False)
        norm_frame_Pred_result = model.predict(source=[frame], conf=0.45, save=False)
        
        if ShowOnFrame_EntireArea_Zone:
            fontFace = cv2.FONT_HERSHEY_SIMPLEX
            fontScale = 0.5
            textThickness = 2
            cv2.polylines(frame, [np.array(ca.entire_area, np.int32)], True, (200, 200, 0), 2) # Entire Area

        PX_convertToFloat_BirdV = pd.DataFrame(bird_eye_Pred_result[0].boxes.data).astype("float")
        PX_convertToNumpy_NormV = norm_frame_Pred_result[0].numpy()

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
        navigateRobotCls = [0] * len(Area)
        binary_map = [# 0  1  2  3  4  5  6  7
                        1, 1, 1, 1, 1, 1, 1, 1, # 0
                        1, 1, 0, 1, 1, 0, 1, 1, # 1
                        1, 1, 1, 1, 1, 1, 1, 1, # 2
                        1, 1, 1, 1, 1, 1, 1, 1, # 3
                        1, 1, 0, 1, 1, 0, 1, 1, # 4
                        1, 1, 1, 1, 1, 1, 1, 1, # 5
                    ]
        robot_default_location = [0, 5]
        sum_of_cls = 0
        getRobot_index = None

        if len(PX_convertToNumpy_NormV) != 0 and ShowOnFrame_BoundingBoxAndClsID:
            Other_Cls_Color = (220, 150, 160)
            robot_color = (200, 55, 10)
            text_color = (255, 255, 255)
            for i in range(len(norm_frame_Pred_result[0])):
                boxes = norm_frame_Pred_result[0].boxes
                box = boxes[i]  # returns one box
                clsID = box.cls.numpy()[0]
                conf = box.conf.numpy()[0]
                bb = box.xyxy.numpy()[0]
                cls_name = class_list[int(clsID)]

                font = cv2.FONT_HERSHEY_COMPLEX

                if cls_name == "robot":
                    cv2.putText(frame, cls_name + " " + str(round(conf, 3)) + "%", (int(bb[0]), int(bb[1]) - 10), font, fontScale=0.5, color=text_color, thickness=2)
                    cv2.rectangle(frame, (int(bb[0]), int(bb[1])), (int(bb[2]), int(bb[3])), robot_color, 2)
                else:
                    cv2.putText(frame, cls_name + " " + str(round(conf, 3)) + "%", (int(bb[0]), int(bb[1]) - 10), font, fontScale=0.5, color=text_color, thickness=2)
                    cv2.rectangle(frame, (int(bb[0]), int(bb[1])), (int(bb[2]), int(bb[3])), Other_Cls_Color, 2)
        
        for index_, row in PX_convertToFloat_BirdV.iterrows():
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

            for area_indx in range(len(Area)):
                if PolygonTest(Area=Area[area_indx], XY=cls_center_pnt) >= 0:
                    Other_Cls_Color = (220, 150, 160)
                    robot_color = (200, 55, 10)
                    list_to_append = path_lists[area_indx]
                    list_to_append.append(cls_center_x)
                    
                    if class_ID_name == "robot":
                        if ShowOnFrame_BoundingBoxAndClsID:
                            boundingBox_ClsID_display(Frame=transform_frame, Rec_pos=rec_pos, Color=robot_color, Text=clsID_and_Conf, Text_pos=text_pos)
                        cv2.circle(transform_frame, cls_center_pnt, 5, robot_color, -1)
                        getRobot_index = area_indx
                        navigateRobotCls[area_indx] = 'R'
                    else:
                        if ShowOnFrame_BoundingBoxAndClsID:
                            boundingBox_ClsID_display(Frame=transform_frame, Rec_pos=rec_pos, Color=Other_Cls_Color, Text=clsID_and_Conf, Text_pos=text_pos)
                        cv2.circle(transform_frame, cls_center_pnt, 5, Other_Cls_Color, -1)
                        binary_map[area_indx] = 0 if len(list_to_append) > 0 else 1
                    # sum_of_cls += len(list_to_append)
        
        directional_format = []
        shortest_path_tuple_format = []
        Robot_Current_Location = navigateRobotLocation(navigateRobotCls, frame_row, frame_column, 'R') if getRobot_index != None else None
        directional_format = []
        target_area = None

        currentTime = time.time()
        if startTime != 0 and (currentTime - startTime) >= interval_reset: # 10 secs
            Table_A_Flag = False
            Table_B_Flag = False
            Table_C_Flag = False
            Table_D_Flag = False
            startTime = 0

        if Robot_Current_Location != None:
            if Table_A_Flag:
                TableA_listOf_areaXY = ca.TableA_nearArea
                numberOfCls_eachArea = [len(PL_02), len(PL_23), len(PL_32), len(PL_20)]
                target_area = getTheTargetArea(TableA_listOf_areaXY, numberOfCls_eachArea, Robot_Current_Location, max_cls_on_area, DEBUG_CMD)

            elif Table_B_Flag:
                TableB_listOf_areaXY = ca.TableB_nearArea
                numberOfCls_eachArea = [len(PL_06), len(PL_27), len(PL_36), len(PL_24)]
                target_area = getTheTargetArea(TableB_listOf_areaXY, numberOfCls_eachArea, Robot_Current_Location, max_cls_on_area, DEBUG_CMD)

            elif Table_C_Flag:
                TableC_listOf_areaXY = ca.TableC_nearArea
                numberOfCls_eachArea = [len(PL_32), len(PL_43), len(PL_52), len(PL_40)]
                target_area = getTheTargetArea(TableC_listOf_areaXY, numberOfCls_eachArea, Robot_Current_Location, max_cls_on_area, DEBUG_CMD)

            elif Table_D_Flag:
                TableD_listOf_areaXY = ca.TableD_nearArea
                numberOfCls_eachArea = [len(PL_36), len(PL_47), len(PL_56), len(PL_44)]
                target_area = getTheTargetArea(TableD_listOf_areaXY, numberOfCls_eachArea, Robot_Current_Location, max_cls_on_area, DEBUG_CMD)
        
        if target_area != None and Robot_Current_Location != None:
            _matrix = ConvertToMatrixOfArray(binary_map, frame_row, frame_column, False)
            shortest_path = cd.CreatePath(Robot_Current_Location, target_area, _matrix, DEBUG_CMD)
            shortest_path_tuple_format = ConvertToListOfTuple(shortest_path)

        if DEBUG_CMD:
            print(f"A Top: {len(PL_02)} Right: {len(PL_23)} Bottom: {len(PL_32)} Left: {len(PL_20)}")
            print(f"B Top: {len(PL_06)} Right: {len(PL_27)} Bottom: {len(PL_36)} Left: {len(PL_24)}")
            print(f"C Top: {len(PL_32)} Right: {len(PL_43)} Bottom: {len(PL_52)} Left: {len(PL_40)}")
            print(f"D Top: {len(PL_36)} Right: {len(PL_47)} Bottom: {len(PL_56)} Left: {len(PL_44)}\n")

            print(f"target area: {target_area}\n")

            print("======= Binary Map =======\n")
            for i in range(len(Area)):
                print(f"{binary_map[i]} ", end="")
                if (i+1) % 8 == 0:
                    print()
            print("\n======= Binary Map End =======\n")
        
        if DEBUG_FRAME:
            if ShowOnFrame_Zones:
                warningColor = (0, 0, 255)
                defaultColor = (0, 0, 0)
                for ind in range(len(Area)):
                    cv2.polylines(transform_frame, [np.array(Area[ind], np.int32)], True, defaultColor, 2)
                    list_to_check = path_lists[ind]
                    sum_of_cls += len(list_to_check) if len(list_to_check) > 0 else 0

                if ShowOnFrame_ObstacledZones:
                    for ind in range(len(Area)):
                        if not binary_map[ind]:
                            cv2.polylines(transform_frame, [np.array(Area[ind], np.int32)], True, warningColor, 2)

                if ShowOnFrame_RobotPathZones and target_area != None and Robot_Current_Location != None:
                    arrayOfNumb = []
                    for indexOf_1Darray in range(len(Area)):
                        arrayOfNumb.append(indexOf_1Darray)
                    matrixOfNumb = ConvertToMatrixOfArray(arrayOfNumb, frame_row, frame_column, False)
                    listOfPath_Index = []
                    # Convert from X and Y to ROWs and COLs
                    for coord in shortest_path_tuple_format:
                        ROW = coord[1]
                        COL = coord[0]
                        listOfPath_Index.append(matrixOfNumb[ROW][COL])
                    row_sArea = Robot_Current_Location[1]
                    col_sArea = Robot_Current_Location[0]
                    row_eArea = target_area[1]
                    col_eArea = target_area[0]

                    Start_Area = matrixOfNumb[row_sArea][col_sArea]
                    End_Area = matrixOfNumb[row_eArea][col_eArea]
                    length_ = len(listOfPath_Index)
                    listOfPath_Index.sort()
                    start_color = (255, 0, 0)
                    end_color = (255, 220, 0)
                    path_color = (0, 255, 0)
                    colors = [start_color, end_color, path_color]

                    increment_index = 0
                    for ind in range(len(Area)):
                        if increment_index != length_:
                            if listOfPath_Index[increment_index] == ind:
                                cv2.polylines(transform_frame, [np.array(Area[ind], np.int32)], True, colors[2], thickness=3)
                                increment_index += 1
                    cv2.polylines(transform_frame, [np.array(Area[Start_Area], np.int32)], True, colors[0], thickness=5)
                    cv2.polylines(transform_frame, [np.array(Area[End_Area], np.int32)], True, colors[1], thickness=5)
        
            rob_loc = Robot_Current_Location if Robot_Current_Location != None else "No robot found!"
            if target_area != None:
                directional_format = cd.convertPathToDirection(shortest_path_tuple_format, row_max=6, col_max=8, display=False)

                cv2.putText(frame, f"Target Area: {target_area}", (50, 35), fontFace=fontFace, fontScale=fontScale, color=(255, 255, 255), thickness=textThickness)
                cv2.putText(frame, f"Robot Location: {rob_loc}", (50, 35+35), fontFace=fontFace, fontScale=fontScale, color=(255, 255, 255), thickness=textThickness)
                cv2.putText(frame, f"Pathway: {shortest_path_tuple_format}", (50, 720-(35+35)), fontFace=fontFace, fontScale=fontScale, color=(255, 255, 255), thickness=textThickness)
                cv2.putText(frame, f"Direction: {directional_format}", (50, 720-35), fontFace=fontFace, fontScale=fontScale, color=(255, 255, 255), thickness=textThickness)
            else:
                cv2.putText(frame, f"Target Area: None", (50, 35), fontFace=fontFace, fontScale=fontScale, color=(255, 255, 255), thickness=textThickness)
                cv2.putText(frame, f"Robot Location: {rob_loc}", (50, 35+35), fontFace=fontFace, fontScale=fontScale, color=(255, 255, 255), thickness=textThickness)
                cv2.putText(frame, f"Pathway: None", (50, 720-(35+35)), fontFace=fontFace, fontScale=fontScale, color=(255, 255, 255), thickness=textThickness)
                cv2.putText(frame, f"Direction: None", (50, 720-35), fontFace=fontFace, fontScale=fontScale, color=(255, 255, 255), thickness=textThickness)

            for ind in range(len(Area)):
                if ShowOnFrame_Binary:
                    cv2.putText(frame, f"{binary_map[ind]}", _center_X_[ind], fontFace=fontFace, fontScale=fontScale, color=(255, 255, 255), thickness=textThickness)
                if ShowOnFrame_NumOfClass:
                    cv2.putText(frame, f"Total Class: {sum_of_cls}", (1100, 35), fontFace=fontFace, fontScale=fontScale, color=(255, 255, 255), thickness=textThickness)
                if ShowOnFrame_IndexNumOfEveryArea:
                    cv2.putText(frame, f"{ind}", _bottom_left_corner_[ind], fontFace=fontFace, fontScale=fontScale, color=(255, 255, 255), thickness=textThickness)

            if ShowOnFrame_EntireArea_Zone:
                if target_area != None:
                    directional_format = cd.convertPathToDirection(shortest_path_tuple_format, row_max=6, col_max=8)

                    cv2.putText(transform_frame, f"Target Area: {target_area}", (50, 35), fontFace=fontFace, fontScale=fontScale, color=(255, 255, 255), thickness=textThickness)
                    cv2.putText(transform_frame, f"Robot Location: {rob_loc}", (50, 35+35), fontFace=fontFace, fontScale=fontScale, color=(255, 255, 255), thickness=textThickness)
                    cv2.putText(transform_frame, f"Pathway: {shortest_path_tuple_format}", (50, 720-(35+35)), fontFace=fontFace, fontScale=fontScale, color=(255, 255, 255), thickness=textThickness)
                    cv2.putText(transform_frame, f"Direction: {directional_format}", (50, 720-35), fontFace=fontFace, fontScale=fontScale, color=(255, 255, 255), thickness=textThickness)
                else:
                    cv2.putText(transform_frame, f"Target Area: None", (50, 35), fontFace=fontFace, fontScale=fontScale, color=(255, 255, 255), thickness=textThickness)
                    cv2.putText(transform_frame, f"Robot Location: {rob_loc}", (50, 35+35), fontFace=fontFace, fontScale=fontScale, color=(255, 255, 255), thickness=textThickness)
                    cv2.putText(transform_frame, f"Pathway: None", (50, 720-(35+35)), fontFace=fontFace, fontScale=fontScale, color=(255, 255, 255), thickness=textThickness)
                    cv2.putText(transform_frame, f"Direction: None", (50, 720-35), fontFace=fontFace, fontScale=fontScale, color=(255, 255, 255), thickness=textThickness)

                for ind in range(len(Area)):
                    if ShowOnFrame_Binary:
                        cv2.putText(transform_frame, f"{binary_map[ind]}", _center_X_[ind], fontFace=fontFace, fontScale=fontScale, color=(255, 255, 255), thickness=textThickness)
                    if ShowOnFrame_NumOfClass:
                        cv2.putText(transform_frame, f"Total Class: {sum_of_cls}", (1100, 35), fontFace=fontFace, fontScale=fontScale, color=(255, 255, 255), thickness=textThickness)
                    if ShowOnFrame_IndexNumOfEveryArea:
                        cv2.putText(transform_frame, f"{ind}", _bottom_left_corner_[ind], fontFace=fontFace, fontScale=fontScale, color=(255, 255, 255), thickness=textThickness)
            
        if ShowTransformFrame:
            cv2.imshow(transform_frame_name, transform_frame)
        if ShowNormalFrame:
            cv2.imshow(window_frame_name, frame)

        if cv2.waitKey(1) & 0xFF == 27: # ESC
            break


    cap.release()
    cv2.destroyAllWindows()
    if ARDUINO:
        arduino.close()
    
if __name__ == "__main__":
    main()