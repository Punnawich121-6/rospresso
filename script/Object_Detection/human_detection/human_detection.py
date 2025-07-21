import torch
import cv2
import numpy as np
from tensorflow.keras.models import load_model # type: ignore
from sklearn.cluster import KMeans

model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)
model.eval()
device = 'cuda' if torch.cuda.is_available() else 'cpu'
model.to(device)

cap = cv2.VideoCapture(0)

height_ = 480
width_ = 640

while True:
    ret, frame = cap.read()

    height, width, channels = frame.shape
    print(f"h:{height}, w:{width}")

    if not ret:
        break

    img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    results = model(img_rgb)

    for *box, conf, cls in results.xyxy[0]:
        class_name = model.names[int(cls)]
        if class_name == 'person':
            x1, y1, x2, y2 = map(int, box)

            print("pose : ", x1, y1, x2, y2 )
            center_x = (x1 + x2) // 2
            center_y = (y1 + y2) // 2

            if center_x > width_ // 2:
                cv2.circle(frame, (int(width_ / 4), int(height_ / 2)), 20, (0, 255, 0), -1)
            elif center_x < width_ // 2:
                cv2.circle(frame, (int((width_ / 4) * 3), int(height_ / 2)), 20, (0, 255, 0), -1)
            else:
                cv2.circle(frame, (int(width_ / 2), int(height_ / 2)), 20, (0, 255, 0), -1)
                    
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0,255,0), 2)

            # Center Line
            # cv2.line(frame, (center_x, y1), (center_x, y1 + x1), (0, 0, 255), 2)

    cv2.imshow("Human Detection", frame)
 
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
