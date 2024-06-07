
import cv2

import os
import time

import playsound

from gtts import gTTS 
language = 'en'

#cap = cv2.VideoCapture('http://100.73.75.23:8080/video')
cap = cv2.VideoCapture(0)

while True:
    success, img = cap.read()
    with open('coco.names', 'r') as f:
        classes = f.read().splitlines()
    net = cv2.dnn.readNetFromDarknet('yolov4.cfg', 'yolov4.weights')
    model = cv2.dnn_DetectionModel(net)
    model.setInputParams(scale=1/255, size=(416, 416), swapRB=True)
    classIds, scores, boxes = model.detect(img, confThreshold=0.6, nmsThreshold=0.4)
    for (classId, score, box) in zip(classIds, scores, boxes):
        cv2.rectangle(img, (box[0], box[1]), (box[0] + box[2], box[1] + box[3]), color=(0, 255, 0), thickness=2)
        mtext = classes[classId]
        myobj = gTTS(text=mtext, lang=language, slow=False) 
        myobj.save("obj.mp3")
        playsound.playsound("obj.mp3")
        os.remove("obj.mp3")
        cv2.putText(img, mtext, (box[0], box[1] - 5), cv2.FONT_HERSHEY_SIMPLEX, 1, color=(0, 255, 0), thickness=2)
    cv2.imshow('Image', cv2.resize(img,(600,500),interpolation = cv2.INTER_CUBIC))


    if cv2.waitKey(1) == ord('q'):
        cv2.destroyAllWindows()
        break
    
cap.release()
