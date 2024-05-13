import cv2
from ultralytics import YOLO
import cvzone
import numpy as np
import pandas as pd
# import serial

ShowOverlayedTexts_TargetAreas = False
ShowOverlayedText_NumOfClass = True
ShowOverlayedZones = True
ShowOverlayedClsBndBox = True

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

def cornerRect_and_putTextRect(Frame, Rec_pos, Color, Text, Text_pos):
    Frame = cvzone.cornerRect(Frame, Rec_pos, l=0, t=2, rt=2, colorR=Color, colorC=Color)
    Frame = cvzone.putTextRect(Frame, Text, Text_pos, 1, 1, colorR=Color)
    # return Frame

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
    # middle area
    mid_path_area = [(469, 318), (469, 492), (816, 492), (816, 318)]
    Mleft_path_area = [(62, 318), (50, 492), (464, 492), (464, 318)]
    Mright_path_area = [(821, 318), (821, 492), (1160, 492), (1125, 318)]
    Mbottom_path_area = [(469, 497), (469, 698), (816, 698), (816, 497)]
    Mtop_path_area = [(469, 100), (469, 313), (816, 313), (816, 100)]
    # corner area
    Tleft_path_area = [(80, 100), (62, 313), (320, 313), (320, 212), (464, 212), (464, 100)]
    Bleft_path_area =  [(50, 497), (39, 698), (464, 698), (464, 580), (320, 580), (320, 497)]
    Bright_path_area = [(821, 585), (821, 698), (1180, 698), (1160, 497), (973, 497), (973, 585)]
    Tright_path_area = [(821, 100), (821, 212), (968, 212), (968, 313), (1125, 313), (1080, 100)]

    tableA_pos = (347, 229)
    tableB_pos = (895, 229)
    tableC_pos = (347, 497)
    tableD_pos = (895, 497)

    pointA_pos = (233, 178)
    pointB_pos = (636, 178)
    pointC_pos = (1005, 178)
    pointD_pos = (233, 400)
    pointE_pos = (636, 400)
    pointF_pos = (1005, 400)
    pointG_pos = (233, 652)
    pointH_pos = (639, 652)
    pointI_pos = (1005, 652)

    M_LRTBcolor = (237, 227, 90)
    TB_LRcolor = (100, 100, 0)
    mcolor = (0, 200, 250)

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
        Mleft_list = []
        Mright_list = []
        Mbottom_list = []
        Mtop_list = []
        Tleft_list = []
        Bleft_list = []
        Tright_list = []
        Bright_list = []

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

            rec_pos = (x1, y1, w, h)
            text_pos = (x1, y1)
            clsID_and_Conf = f"{class_ID_name} {confidence}%"

            mid_path_result = cv2.pointPolygonTest(np.array(mid_path_area, np.int32), ((cx, cy)), False)
            Mleft_path_result = cv2.pointPolygonTest(np.array(Mleft_path_area, np.int32), ((cx, cy)), False)
            Mright_path_result = cv2.pointPolygonTest(np.array(Mright_path_area, np.int32), ((cx, cy)), False)
            Mbottom_path_result = cv2.pointPolygonTest(np.array(Mbottom_path_area, np.int32), ((cx, cy)), False)
            Mtop_path_result = cv2.pointPolygonTest(np.array(Mtop_path_area, np.int32), ((cx, cy)), False)
            Tleft_path_result = cv2.pointPolygonTest(np.array(Tleft_path_area, np.int32), ((cx, cy)), False)
            Bleft_path_result = cv2.pointPolygonTest(np.array(Bleft_path_area, np.int32), ((cx, cy)), False)
            Tright_path_result = cv2.pointPolygonTest(np.array(Tright_path_area, np.int32), ((cx, cy)), False)
            Bright_path_result = cv2.pointPolygonTest(np.array(Bright_path_area, np.int32), ((cx, cy)), False)

            # print("Start ===\n" +
            #       f"Mid = {mid_path_result}\n" +
            #       f"ML = {Mleft_path_result}\n" +
            #       f"MR = {Mright_path_result}\n" +
            #       f"MB = {Mbottom_path_result}\n" +
            #       f"MT = {Mtop_path_result}\n" +
            #       "=== End")

            if mid_path_result >= 0 and ShowOverlayedClsBndBox:
                cornerRect_and_putTextRect( Frame=frame,
                                            Rec_pos=rec_pos,
                                            Color=mcolor,
                                            Text=clsID_and_Conf,
                                            Text_pos=text_pos )
                cv2.circle(frame, (cx, cy), 4, mcolor, -1)
                mid_list.append(cx)
            elif mid_path_result >= 0 and not ShowOverlayedClsBndBox:
                cv2.circle(frame, (cx, cy), 4, mcolor, -1)
                mid_list.append(cx)

            if Mleft_path_result >= 0 and ShowOverlayedClsBndBox:
                cornerRect_and_putTextRect(Frame=frame,
                                               Rec_pos=rec_pos,
                                               Color=M_LRTBcolor,
                                               Text=clsID_and_Conf,
                                               Text_pos=text_pos)
                cv2.circle(frame, (cx, cy), 4, M_LRTBcolor, -1)
                Mleft_list.append(cx)
            elif Mleft_path_result >= 0 and not ShowOverlayedClsBndBox:
                cv2.circle(frame, (cx, cy), 4, M_LRTBcolor, -1)
                Mleft_list.append(cx)
            
            if Mright_path_result >= 0 and ShowOverlayedClsBndBox:
                cornerRect_and_putTextRect(Frame=frame,
                                               Rec_pos=rec_pos,
                                               Color=M_LRTBcolor,
                                               Text=clsID_and_Conf,
                                               Text_pos=text_pos)
                cv2.circle(frame, (cx, cy), 4, M_LRTBcolor, -1)
                Mright_list.append(cx)
            elif Mright_path_result >= 0 and not ShowOverlayedClsBndBox:
                cv2.circle(frame, (cx, cy), 4, M_LRTBcolor, -1)
                Mright_list.append(cx)

            if Mbottom_path_result >= 0 and ShowOverlayedClsBndBox:
                cornerRect_and_putTextRect(Frame=frame,
                                               Rec_pos=rec_pos,
                                               Color=M_LRTBcolor,
                                               Text=clsID_and_Conf,
                                               Text_pos=text_pos)
                cv2.circle(frame, (cx, cy), 4, M_LRTBcolor, -1)
                Mbottom_list.append(cx)
            elif Mbottom_path_result >= 0 and not ShowOverlayedClsBndBox:
                cv2.circle(frame, (cx, cy), 4, M_LRTBcolor, -1)
                Mbottom_list.append(cx)

            if Mtop_path_result >= 0 and ShowOverlayedClsBndBox:
                cornerRect_and_putTextRect(Frame=frame,
                                               Rec_pos=rec_pos,
                                               Color=M_LRTBcolor,
                                               Text=clsID_and_Conf,
                                               Text_pos=text_pos)
                cv2.circle(frame, (cx, cy), 4, M_LRTBcolor, -1)
                Mtop_list.append(cx)
            elif Mtop_path_result >= 0 and not ShowOverlayedClsBndBox:
                cv2.circle(frame, (cx, cy), 4, M_LRTBcolor, -1)
                Mtop_list.append(cx)
            
            if Tleft_path_result >= 0 and ShowOverlayedClsBndBox:
                cornerRect_and_putTextRect(Frame=frame,
                                               Rec_pos=rec_pos,
                                               Color=TB_LRcolor,
                                               Text=clsID_and_Conf,
                                               Text_pos=text_pos)
                cv2.circle(frame, (cx, cy), 4, TB_LRcolor, -1)
                Tleft_list.append(cx)
            elif Tleft_path_result >= 0 and not ShowOverlayedClsBndBox:
                cv2.circle(frame, (cx, cy), 4, TB_LRcolor, -1)
                Tleft_list.append(cx)

            if Bleft_path_result >= 0 and ShowOverlayedClsBndBox:
                cornerRect_and_putTextRect(Frame=frame,
                                               Rec_pos=rec_pos,
                                               Color=TB_LRcolor,
                                               Text=clsID_and_Conf,
                                               Text_pos=text_pos)
                cv2.circle(frame, (cx, cy), 4, TB_LRcolor, -1)
                Bleft_list.append(cx)
            elif Bleft_path_result >= 0 and not ShowOverlayedClsBndBox:
                cv2.circle(frame, (cx, cy), 4, TB_LRcolor, -1)
                Bleft_list.append(cx)

            if Tright_path_result >= 0 and ShowOverlayedClsBndBox:
                cornerRect_and_putTextRect(Frame=frame,
                                               Rec_pos=rec_pos,
                                               Color=TB_LRcolor,
                                               Text=clsID_and_Conf,
                                               Text_pos=text_pos)
                cv2.circle(frame, (cx, cy), 4, TB_LRcolor, -1)
                Tright_list.append(cx)
            elif Tright_path_result >= 0 and not ShowOverlayedClsBndBox:
                cv2.circle(frame, (cx, cy), 4, TB_LRcolor, -1)
                Tright_list.append(cx)

            if Bright_path_result >= 0 and ShowOverlayedClsBndBox:
                cornerRect_and_putTextRect(Frame=frame,
                                               Rec_pos=rec_pos,
                                               Color=TB_LRcolor,
                                               Text=clsID_and_Conf,
                                               Text_pos=text_pos)
                cv2.circle(frame, (cx, cy), 4, TB_LRcolor, -1)
                Bright_list.append(cx)
            elif Bright_path_result >= 0 and not ShowOverlayedClsBndBox:
                cv2.circle(frame, (cx, cy), 4, TB_LRcolor, -1)
                Bright_list.append(cx)

        numOfClsFnd_midPath = len(mid_list)
        numOfClsFnd_MleftPath = len(Mleft_list)
        numOfClsFnd_MrightPath = len(Mright_list)
        numOfClsFnd_MbottomPath = len(Mbottom_list)
        numOfClsFnd_MtopPath = len(Mtop_list)
        numOfClsFnd_TleftPath = len(Tleft_list)
        numOfClsFnd_BleftPath = len(Bleft_list)
        numOfClsFnd_TrightPath = len(Tright_list)
        numOfClsFnd_BrightPath = len(Bright_list)

        if ShowOverlayedText_NumOfClass:
            cvzone.putTextRect(frame, f"Middle Zone: {numOfClsFnd_midPath}", pos=(50, 60), scale=1, thickness=2, colorR=(100, 100, 0))
            cvzone.putTextRect(frame, f"M-Left Zone: {numOfClsFnd_MleftPath} || M-Right Zone: {numOfClsFnd_MrightPath}", pos=(50, 60+35), scale=1, thickness=2, colorR=(100, 100, 0))
            cvzone.putTextRect(frame, f"M-Top Zone: {numOfClsFnd_MtopPath} || M-Bottom Zone: {numOfClsFnd_MbottomPath}", pos=(50, 60+35+35), scale=1, thickness=2, colorR=(100, 100, 0))
            cvzone.putTextRect(frame, f"T-Left Zone: {numOfClsFnd_TleftPath} || T-Right Zone: {numOfClsFnd_TrightPath}", pos=(50, 60+35+35+35), scale=1, thickness=2, colorR=(100, 100, 0))
            cvzone.putTextRect(frame, f"B-Left Zone: {numOfClsFnd_BleftPath} || B-Right Zone: {numOfClsFnd_BrightPath}", pos=(50, 60+35+35+35+35), scale=1, thickness=2, colorR=(100, 100, 0))

        if ShowOverlayedZones:
            Redcolor = (0, 0, 255)

            if numOfClsFnd_midPath > 0:
                cv2.polylines(frame, [np.array(mid_path_area, np.int32)], True, Redcolor, 2)
            else:
                cv2.polylines(frame, [np.array(mid_path_area, np.int32)], True, mcolor, 2)

            if numOfClsFnd_MleftPath > 0:
                cv2.polylines(frame, [np.array(Mleft_path_area, np.int32)], True, Redcolor, 2)
            else:
                cv2.polylines(frame, [np.array(Mleft_path_area, np.int32)], True, M_LRTBcolor, 2)

            if numOfClsFnd_MrightPath > 0:
                cv2.polylines(frame, [np.array(Mright_path_area, np.int32)], True, Redcolor, 2)
            else:
                cv2.polylines(frame, [np.array(Mright_path_area, np.int32)], True, M_LRTBcolor, 2)

            if numOfClsFnd_MbottomPath > 0:
                cv2.polylines(frame, [np.array(Mbottom_path_area, np.int32)], True, Redcolor, 2)
            else:
                cv2.polylines(frame, [np.array(Mbottom_path_area, np.int32)], True, M_LRTBcolor, 2)
            
            if numOfClsFnd_MtopPath > 0:
                cv2.polylines(frame, [np.array(Mtop_path_area, np.int32)], True, Redcolor, 2)
            else:
                cv2.polylines(frame, [np.array(Mtop_path_area, np.int32)], True, M_LRTBcolor, 2)


            if numOfClsFnd_TleftPath > 0:
                cv2.polylines(frame, [np.array(Tleft_path_area, np.int32)], True, Redcolor, 2)
            else:
                cv2.polylines(frame, [np.array(Tleft_path_area, np.int32)], True, TB_LRcolor, 2)

            if numOfClsFnd_BleftPath > 0:
                cv2.polylines(frame, [np.array(Bleft_path_area, np.int32)], True, Redcolor, 2)
            else:
                cv2.polylines(frame, [np.array(Bleft_path_area, np.int32)], True, TB_LRcolor, 2)

            if numOfClsFnd_TrightPath > 0:
                cv2.polylines(frame, [np.array(Tright_path_area, np.int32)], True, Redcolor, 2)
            else:
                cv2.polylines(frame, [np.array(Tright_path_area, np.int32)], True, TB_LRcolor, 2)

            if numOfClsFnd_BrightPath > 0:
                cv2.polylines(frame, [np.array(Bright_path_area, np.int32)], True, Redcolor, 2)
            else:                
                cv2.polylines(frame, [np.array(Bright_path_area, np.int32)], True, TB_LRcolor, 2)

        if ShowOverlayedTexts_TargetAreas:
            cv2.putText(frame, f"Table A {tableA_pos}", tableA_pos, fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.5, color=(255, 255, 255), thickness = 2)
            cv2.putText(frame, f"Table B {tableB_pos}", tableB_pos, fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.5, color=(255, 255, 255), thickness = 2)
            cv2.putText(frame, f"Table C {tableC_pos}", tableC_pos, fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.5, color=(255, 255, 255), thickness = 2)
            cv2.putText(frame, f"Table D {tableD_pos}", tableD_pos, fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.5, color=(255, 255, 255), thickness = 2)

            cv2.circle(frame, pointA_pos, radius=5, color=(0, 255, 0), thickness=-1)
            cv2.circle(frame, pointB_pos, radius=5, color=(0, 255, 0), thickness=-1)
            cv2.circle(frame, pointC_pos, radius=5, color=(0, 255, 0), thickness=-1)
            cv2.circle(frame, pointD_pos, radius=5, color=(0, 255, 0), thickness=-1)
            cv2.circle(frame, pointE_pos, radius=5, color=(0, 255, 0), thickness=-1)
            cv2.circle(frame, pointF_pos, radius=5, color=(0, 255, 0), thickness=-1)
            cv2.circle(frame, pointG_pos, radius=5, color=(0, 255, 0), thickness=-1)
            cv2.circle(frame, pointH_pos, radius=5, color=(0, 255, 0), thickness=-1)
            cv2.circle(frame, pointI_pos, radius=5, color=(0, 255, 0), thickness=-1)

            cv2.putText(frame, f"A {pointA_pos}", (pointA_pos[0] - 35, pointA_pos[1] - 15), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.5, color=(255, 255, 255), thickness = 2)
            cv2.putText(frame, f"B {pointB_pos}", (pointB_pos[0] - 35, pointB_pos[1] - 15), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.5, color=(255, 255, 255), thickness = 2)
            cv2.putText(frame, f"C {pointC_pos}", (pointC_pos[0] - 35, pointC_pos[1] - 15), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.5, color=(255, 255, 255), thickness = 2)
            cv2.putText(frame, f"D {pointD_pos}", (pointD_pos[0] - 35, pointD_pos[1] - 15), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.5, color=(255, 255, 255), thickness = 2)
            cv2.putText(frame, f"E {pointE_pos}", (pointE_pos[0] - 35, pointE_pos[1] - 15), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.5, color=(255, 255, 255), thickness = 2)
            cv2.putText(frame, f"F {pointF_pos}", (pointF_pos[0] - 35, pointF_pos[1] - 15), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.5, color=(255, 255, 255), thickness = 2)
            cv2.putText(frame, f"G {pointG_pos}", (pointG_pos[0] - 35, pointG_pos[1] - 15), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.5, color=(255, 255, 255), thickness = 2)
            cv2.putText(frame, f"H {pointH_pos}", (pointH_pos[0] - 35, pointH_pos[1] - 15), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.5, color=(255, 255, 255), thickness = 2)
            cv2.putText(frame, f"I {pointI_pos}", (pointI_pos[0] - 35, pointI_pos[1] - 15), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.5, color=(255, 255, 255), thickness = 2)

        cv2.imshow("Video Frame", frame)
        # cv2.imshow("Video Frame1", frame1)

        if cv2.waitKey(1) & 0xFF == 27: # ESC
            break


    cap.release()
    # cap1.release()
    cv2.destroyAllWindows()
    

if __name__ == "__main__":
    main()