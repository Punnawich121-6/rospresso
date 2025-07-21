
import cv2
import tensorflow
import keras 
from PIL import Image

face_cascade = "script/Object_Detection/face_detection/haarcascade_frontalface_default.xml"

webcam = cv2.VideoCapture(0)
while True:
 
    success, image_bgr = webcam.read()

    image_bw = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)

    face_classifier = cv2.CascadeClassifier(face_cascade)
    faces = face_classifier.detectMultiScale(image_bw)

    print(f'There are {len(faces)} faces found.')

    for face in faces:
        x, y, w, h = face
        cv2.rectangle(image_bgr, (x, y), (x+w, y+h), (0, 255, 0), 2)
        # cv2.imwrite(f'src/bot_pkg/script/ObjectDetection.py/face_detect/mask/mask_{count}.jpg',image_org[y:y+h,x:x+w])


    k = cv2.waitKey(5) & 0xFF
    if k==27:  # ESC
        break

cv2.imshow("Faces found", image_bgr)
cv2.waitKey(1) 
