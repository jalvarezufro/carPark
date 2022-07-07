import cv2
import pickle
import cvzone
import numpy as np
import paho.mqtt.client as mqtt
import json

# Video feed
cap = cv2.VideoCapture('videoUfro.mp4')

# thingsboard connection
THINGSBOARD_HOST = 'demo.thingsboard.io'
ACCESS_TOKEN = '1XN8uGGhovnO274Jg4QP'
client = mqtt.Client()
client.username_pw_set(ACCESS_TOKEN)
client.connect(THINGSBOARD_HOST, 1883, 60)
print("conexion exitosa")

with open('CarParkPos', 'rb') as f:
    posList = pickle.load(f)
    # print(len(posList))
width, height = 80, 130

spaceJson = {}


def checkParkingSpace(imgPro):
    spaceCounter = 0
    for i, pos in enumerate(posList):
        a, b, c, d = pos
        x1 = min(a[0], b[0], c[0], d[0])
        x2 = max(a[0], b[0], c[0], d[0])
        y1 = min(a[1], b[1], c[1], d[1])
        y2 = max(a[1], b[1], c[1], d[1])

        imgCrop = imgPro[y1:y2, x1:x2]
        # cv2.imshow(str(x1 * y1), imgCrop)
        count = cv2.countNonZero(imgCrop)

        if count < 1000:
            color = (0, 255, 0)
            thickness = 5
            spaceCounter += 1
            spaceJson["pos" + str(i)] = 0
        else:
            color = (0, 0, 255)
            thickness = 2
            spaceJson["pos" + str(i)] = 1

        pts = np.array(pos, np.int32)
        isClosed = True

        image = cv2.polylines(img, [pts],
                              isClosed, color, thickness)
        cvzone.putTextRect(img, str(count), (x1, y1 - 3), scale=1,
                           thickness=2, offset=0, colorR=color)

    # print(spaceJson)
    cvzone.putTextRect(img, f'Free: {spaceCounter}/{len(posList)}', (100, 50), scale=3,
                       thickness=5, offset=20, colorR=(0, 200, 0))


client.loop_start()

while True:

    # if cap.get(cv2.CAP_PROP_POS_FRAMES) == cap.get(cv2.CAP_PROP_FRAME_COUNT):
    #     cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
    success, img = cap.read()

    if success:
        imgGray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        imgBlur = cv2.GaussianBlur(imgGray, (3, 3), 1)
        imgThreshold = cv2.adaptiveThreshold(imgBlur, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                             cv2.THRESH_BINARY_INV, 25, 16)
        imgMedian = cv2.medianBlur(imgThreshold, 5)
        kernel = np.ones((3, 3), np.uint8)
        imgDilate = cv2.dilate(imgMedian, kernel, iterations=1)

        checkParkingSpace(imgDilate)
        client.publish('v1/devices/me/telemetry', json.dumps(spaceJson), 1)
        print("data enviada ", spaceJson)
        # cv2.imshow("Image", img)
        # cv2.imshow("ImageBlur", imgBlur)
        # cv2.imshow("ImageThres", imgMedian)
        cv2.waitKey(10)
    else:
        print('no video')
        cap = cv2.VideoCapture('b.mp4')
        continue

client.loop_stop()
client.disconnect()
