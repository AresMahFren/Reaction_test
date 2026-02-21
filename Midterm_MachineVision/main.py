import cv2
import mediapipe as mp
import time
import random
import math
from cvzone.HandTrackingModule import HandDetector

detector = HandDetector(detectionCon=0.8, maxHands=2)
mp_pose = mp.solutions.pose
mp_drawing = mp.solutions.drawing_utils

# ---------- SETTINGS ----------
width, height = 1280, 720
angle_threshold = 160
min_visibility = 0.6

# ---------- CAMERA ----------
cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
cap.set(3, width)
cap.set(4, height)

mp_pose = mp.solutions.pose
pose = mp_pose.Pose(
    min_detection_confidence=0.7,
    min_tracking_confidence=0.7
)

# ---------- FUNCTIONS ----------
def calculate_angle(a, b, c):
    angle = math.degrees(
        math.atan2(c.y - b.y, c.x - b.x) -
        math.atan2(a.y - b.y, a.x - b.x)
    )
    if angle < 0:
        angle += 360
    if angle > 180:
        angle = 360 - angle
    return angle

def arm_straight(landmarks, side):
    elbow = landmarks[14]
    wrist = landmarks[16]

    if elbow.visibility < min_visibility or wrist.visibility < min_visibility:
        return False

    # Check if wrist is significantly above elbow
    return wrist.y < elbow.y - 0.05

    angle = calculate_angle(elbow, wrist)

    return angle > angle_threshold

def arm_visible(landmarks):
    return landmarks[16].visibility > min_visibility

# ---------- GAME VARIABLES ----------
state = "WAITING"
start_time = 0
go_time = 0
reaction_time = 0
delay = 0
angle_threshold = 160
angle_threshold = 145
cv2.namedWindow("Reaction Game", cv2.WND_PROP_FULLSCREEN)
cv2.setWindowProperty("Reaction Game", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

# ---------- MAIN LOOP ----------
while True:
    ret, frame = cap.read()
    if not ret:
        break

    frame = cv2.flip(frame, 1)
    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    results = pose.process(rgb)

    # Detect hands
    hands, frame = detector.findHands(frame, draw=False)

    # ===============================
    # POSE DETECTION
    # ===============================
    if results.pose_landmarks:
        landmarks = results.pose_landmarks.landmark

        arm_connections = [
            (14, 16)  # RIGHT elbow → RIGHT wrist only
        ]

        for connection in arm_connections:
            start = landmarks[connection[0]]
            end = landmarks[connection[1]]

            if start.visibility > 0.6 and end.visibility > 0.6:
                x1, y1 = int(start.x * width), int(start.y * height)
                x2, y2 = int(end.x * width), int(end.y * height)

                cv2.line(frame, (x1, y1), (x2, y2), (0, 255, 0), 5)

        # ===============================
        # GAME STATE LOGIC
        # ===============================

        if state == "WAITING":
            cv2.putText(frame, "Press S to Start",
                        (450, 350), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2)

        elif state == "READY":
            cv2.putText(frame, "X",
                        (600, 350), cv2.FONT_HERSHEY_SIMPLEX, 5, (0,0,255), 10)

            if arm_visible(landmarks):
                state = "WAITING"
                print("False Start!")

            if time.time() - start_time > delay:
                state = "GO"
                go_time = time.time()

        elif state == "GO":
            cv2.putText(frame, "O",
                        (600, 350), cv2.FONT_HERSHEY_SIMPLEX, 5, (0,255,0), 10)

            right_straight = arm_straight(landmarks, "right")

            if right_straight:
                reaction_time = time.time() - go_time
                state = "RESULT"

        elif state == "RESULT":
            cv2.putText(frame, f"Reaction Time: {reaction_time:.3f}s",
                        (400, 350), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0,255,0), 3)
            cv2.putText(frame, "Press R to Retry",
                        (450, 420), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2)

    # ===============================
    # HAND GUN DRAWING (OUTSIDE pose if)
    # ===============================
    if hands:
        hand = hands[0]
        lmList = hand["lmList"]

        cv2.line(frame, lmList[0][:2], lmList[8][:2], (0,255,0), 5)
        cv2.line(frame, lmList[0][:2], lmList[12][:2], (0,255,0), 5)
        cv2.line(frame, lmList[0][:2], lmList[4][:2], (0,255,0), 5)

    cv2.imshow("Reaction Test", frame)

    key = cv2.waitKey(1) & 0xFF

    if key == ord('q'):
        break

    if key == ord('s') and state == "WAITING":
        if results.pose_landmarks:
            landmarks = results.pose_landmarks.landmark
            if not arm_visible(landmarks):
                state = "READY"
                start_time = time.time()
                delay = random.uniform(2, 5)
            else:
                print("Hide your arms first!")

    if key == ord('r'):
        state = "WAITING"

cap.release()
cv2.destroyAllWindows()