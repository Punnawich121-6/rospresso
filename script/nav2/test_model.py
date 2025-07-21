import torch
import cv2

model_path = 'src/bot_pkg/scripts/ObjectDetection/best.pt'
model = torch.hub.load('ultralytics/yolov5', 'custom', path=model_path)
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        print("Can't access to camera !")
        break

    results = model(frame)

    results.render()  
    cv2.imshow("YOLOv5 Detection", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
