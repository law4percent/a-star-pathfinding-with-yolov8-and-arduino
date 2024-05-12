import cv2
import numpy as np
import urllib.request
import serial

url = 'http://192.168.43.219/cam-hi.jpg'

arduino_port = 'COM11'  # Modify this with the appropriate serial port
baud_rate = 9600  # Modify this with the appropriate baud rate

cap = cv2.VideoCapture(url)
whT = 320
confThreshold = 0.5
nmsThreshold = 0.3
classesfile = 'coco.names'
classNames = []

with open(classesfile, 'rt') as f:
    classNames = f.read().rstrip('\n').split('\n')

carClassId = classNames.index('car')  # Get the class ID for car

modelConfig = 'yolov3.cfg'
modelWeights = 'yolov3.weights'
net = cv2.dnn.readNetFromDarknet(modelConfig, modelWeights)
net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)

door_opened = False
door_closed = False
flag=0
flag1=0
Ccounter=0 #for counting cars

line1_x1, line1_y1, line1_x2, line1_y2 = 200, 0, 200, 600  # Coordinates for line 1 (x1, y1, x2, y2)
line2_x1, line2_y1, line2_x2, line2_y2 = 600, 0, 600, 600  # Coordinates for line 2 (x1, y1, x2, y2)

# Establish serial connection with Arduino Nano
arduino = serial.Serial(arduino_port, baud_rate, timeout=1)

def send_servo_command(angle):
    command = f"{angle}\n"  # Append newline character to the command
    arduino.write(command.encode('utf-8'))


def findObject(outputs, img):
    global door_opened, door_closed,flag, Ccounter# Declare the variables as global

    hT, wT, cT = img.shape
    bbox = []
    classIds = []
    confs = []

    for output in outputs:
        for det in output:
            scores = det[5:]
            classId = np.argmax(scores)
            confidence = scores[classId]

            if classId == carClassId and confidence > confThreshold:
                w, h = int(det[2] * wT), int(det[3] * hT)
                x, y = int((det[0] * wT) - w / 2), int((det[1] * hT) - h / 2)
                bbox.append([x, y, w, h])
                classIds.append(classId)
                confs.append(float(confidence))

             

    indices = cv2.dnn.NMSBoxes(bbox, confs, confThreshold, nmsThreshold)

    for i in indices:
        i = i[0]
        box = bbox[i]
        x, y, w, h = box[0], box[1], box[2], box[3]
        center_x = x + w // 2
        
        center_y = y + h // 2
        cv2.rectangle(img, (x, y), (x + w, y + h), (255, 0, 255), 2)
        cv2.putText(img, f'{classNames[classIds[i]].upper()} {int(confs[i] * 100)}%', (x, y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)
        cv2.circle(img, (center_x, center_y), 10, (0, 255, 0), -1)  # Draw a dot at the center of the bounding box
        
        if flag == 0 and center_x > line1_x1 and center_x < line2_x1:
            door_opened = True
            flag=1
            print("Door opened")
            send_servo_command(1)  # Send 90 degree command to servo
            
                    
        if flag==1 and center_x > line2_x1:
            door_closed = True
            flag=0
            print("Door closed")
            send_servo_command(2)  # Send 0 degree command to servo
             
        

    # Draw line 1
    cv2.line(img, (line1_x1, line1_y1), (line1_x2, line1_y2), (0, 255, 0), 2)

    # Draw line 2
    cv2.line(img, (line2_x1, line2_y1), (line2_x2, line2_y2), (0, 0, 255), 2)


while True:
    img_resp = urllib.request.urlopen(url)

    imgnp = np.array(bytearray(img_resp.read()), dtype=np.uint8)
    
    im = cv2.imdecode(imgnp, -1)
    
    success, img = cap.read()
    
    blob = cv2.dnn.blobFromImage(im, 1 / 255, (whT, whT), [0, 0, 0], 1, crop=False)

    net.setInput(blob)

    layerNames = net.getLayerNames()

    outputNames = [layerNames[i[0] - 1] for i in net.getUnconnectedOutLayers()]
    
    outputs = net.forward(outputNames)
    
    findObject(outputs, im)
    
    cv2.imshow('Image', im)

    if door_opened and door_closed:
        door_opened = False
        door_closed = False
        
    if cv2.waitKey(1) == ord('q'):
        break

cv2.destroyAllWindows()