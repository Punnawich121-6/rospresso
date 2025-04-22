import cv2
import torch
import mediapipe as mp

# Load YOLOv5 model
model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)

# Init MediaPipe
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(max_num_hands=1, min_detection_confidence=0.7, min_tracking_confidence=0.7)

# Debounce parameters
last_pointed = None
stable_count = 0
DEBOUNCE_FRAMES = 5

cap = cv2.VideoCapture(0)

def intersects(bbox, p1, p2):
    """Check if the ray (p1→p2) intersects the bounding box."""
    x_min, y_min, x_max, y_max = bbox
    # Check line-rectangle intersection (simple approximation)
    if p1[0] > x_max and p2[0] > x_max: return False
    if p1[0] < x_min and p2[0] < x_min: return False
    if p1[1] > y_max and p2[1] > y_max: return False
    if p1[1] < y_min and p2[1] < y_min: return False
    return True

while True:
    ret, frame = cap.read()
    if not ret:
        break
    frame = cv2.flip(frame, 1)
    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    # Detect bottles
    results = model(frame)
    detections = results.pandas().xyxy[0]

    bottles = []
    for row in detections.itertuples():
        if row.name == 'bottle':
            bottles.append({
                'index': len(bottles),
                'bbox': (int(row.xmin), int(row.ymin), int(row.xmax), int(row.ymax)),
                'center_x': int(row.xmin + (row.xmax - row.xmin)/2)
            })

    bottles.sort(key=lambda b: b['center_x'])

    # Draw bottles
    for idx, b in enumerate(bottles):
        x1, y1, x2, y2 = b['bbox']
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(frame, f"Bottle {idx}", (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    # Detect hand & finger
    result = hands.process(rgb)
    pointed_index = None

    if result.multi_hand_landmarks:
        for hand_landmarks in result.multi_hand_landmarks:
            # Get keypoints
            h, w, _ = frame.shape
            p6 = hand_landmarks.landmark[6]  # knuckle of index
            p8 = hand_landmarks.landmark[8]  # tip of index
            x6, y6 = int(p6.x * w), int(p6.y * h)
            x8, y8 = int(p8.x * w), int(p8.y * h)

            # Extend ray
            dx, dy = x8 - x6, y8 - y6
            x_far, y_far = x8 + dx * 5, y8 + dy * 5  # cast long ray

            cv2.circle(frame, (x8, y8), 8, (255, 0, 0), -1)
            cv2.line(frame, (x6, y6), (int(x_far), int(y_far)), (255, 0, 0), 2)

            # Check intersection with each bottle
            for idx, b in enumerate(bottles):
                if intersects(b['bbox'], (x6, y6), (x_far, y_far)):
                    pointed_index = idx
                    break

    # Debounce logic
    if pointed_index == last_pointed:
        stable_count += 1
    else:
        stable_count = 1
        last_pointed = pointed_index

    if stable_count >= DEBOUNCE_FRAMES and pointed_index is not None:
        cv2.putText(frame, f"Pointing to Bottle {pointed_index}", (30, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3)
        print(f"Pointing to Bottle {pointed_index}")

    cv2.imshow("Bottle & Finger Detection (Debounce + Ray)", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
