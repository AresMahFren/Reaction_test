import cv2
import mediapipe as mp
import time

# =============================
# HAND TRACKING SETUP
# =============================
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(max_num_hands=1)
mp_draw = mp.solutions.drawing_utils

# =============================
# QUESTIONS (sample only)
# =============================
questions = [
    ("What is 2 + 2?", ["1", "2", "3", "4"], 3),
    ("Capital of France?", ["Berlin", "Madrid", "Paris", "Rome"], 2),
    ("5 * 3?", ["15", "10", "20", "25"], 0),
    ("Color of sky?", ["Blue", "Green", "Red", "Yellow"], 0),
    ("Python is a?", ["Snake", "Language", "Car", "Food"], 1),
    ("Sun rises in?", ["West", "North", "East", "South"], 2),
    ("Water formula?", ["H2O", "CO2", "O2", "NaCl"], 0),
    ("Earth is?", ["Flat", "Round", "Square", "Triangle"], 1),
    ("Fastest land animal?", ["Lion", "Tiger", "Cheetah", "Dog"], 2),
    ("1+1?", ["1", "2", "3", "4"], 1),
    ("HTML stands for?", ["Hyper Text Markup Language", "Hot Mail", "How to Make Lasagna", "None"], 0),
    ("Largest planet?", ["Earth", "Mars", "Jupiter", "Venus"], 2),
    ("Binary of 2?", ["10", "11", "01", "00"], 0),
    ("Square root of 16?", ["2", "3", "4", "5"], 2),
    ("Final prize?", ["100", "1000", "1M", "10M"], 2)
]

money = [
    "$100", "$200", "$300", "$500", "$1,000",
    "$2,000", "$4,000", "$8,000", "$16,000",
    "$32,000", "$64,000", "$125,000",
    "$250,000", "$500,000", "$1,000,000"
]

# =============================
# FINGER COUNT FUNCTION
# =============================
def count_fingers(hand_landmarks):
    fingers = []

    # Tip IDs
    tips = [4, 8, 12, 16, 20]

    # Thumb
    if hand_landmarks.landmark[tips[0]].x < hand_landmarks.landmark[tips[0] - 1].x:
        fingers.append(1)
    else:
        fingers.append(0)

    # Other fingers
    for i in range(1, 5):
        if hand_landmarks.landmark[tips[i]].y < hand_landmarks.landmark[tips[i] - 2].y:
            fingers.append(1)
        else:
            fingers.append(0)

    return sum(fingers)

# =============================
# GAME VARIABLES
# =============================
cap = cv2.VideoCapture(0)

game_state = "START"
level = 0
current_money = "$0"

last_gesture_time = 0
gesture_delay = 2  # seconds

# =============================
# MAIN LOOP
# =============================
while True:
    success, img = cap.read()
    img = cv2.flip(img, 1)

    rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    result = hands.process(rgb)

    fingers = 0

    if result.multi_hand_landmarks:
        for handLms in result.multi_hand_landmarks:
            mp_draw.draw_landmarks(img, handLms, mp_hands.HAND_CONNECTIONS)
            fingers = count_fingers(handLms)

    current_time = time.time()

    # =============================
    # START SCREEN
    # =============================
    if game_state == "START":
        cv2.putText(img, "SHOW 1 FINGER TO START", (50, 200),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 3)

        if fingers == 1 and current_time - last_gesture_time > gesture_delay:
            game_state = "QUESTION"
            level = 0
            last_gesture_time = current_time

    # =============================
    # QUESTION STATE
    # =============================
    elif game_state == "QUESTION":
        q, choices, correct = questions[level]

        cv2.putText(img, f"Level {level+1} - {money[level]}", (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 0), 2)

        cv2.putText(img, q, (20, 100),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

        for i, choice in enumerate(choices):
            cv2.putText(img, f"{chr(65+i)}: {choice}", (20, 150 + i*40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 200), 2)

        cv2.putText(img, f"Fingers: {fingers}", (400, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)

        if 1 <= fingers <= 4 and current_time - last_gesture_time > gesture_delay:
            if fingers - 1 == correct:
                current_money = money[level]
                level += 1
                last_gesture_time = current_time

                if level == 15:
                    game_state = "WIN"
                else:
                    game_state = "DECISION"
            else:
                game_state = "GAME_OVER"
                last_gesture_time = current_time

    # =============================
    # DECISION STATE
    # =============================
    elif game_state == "DECISION":
        cv2.putText(img, f"You won {current_money}", (50, 150),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 3)

        cv2.putText(img, "1 Finger = Continue", (50, 250),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

        cv2.putText(img, "2 Fingers = Keep Money", (50, 300),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

        if current_time - last_gesture_time > gesture_delay:
            if fingers == 1:
                game_state = "QUESTION"
                last_gesture_time = current_time
            elif fingers == 2:
                game_state = "WIN"
                last_gesture_time = current_time

    # =============================
    # GAME OVER
    # =============================
    elif game_state == "GAME_OVER":
        cv2.putText(img, "WRONG ANSWER!", (100, 200),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 0, 255), 4)

        cv2.putText(img, "You lost everything!", (100, 260),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

        cv2.putText(img, "Show 1 finger to restart", (100, 320),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

        if fingers == 1:
            game_state = "START"

    # =============================
    # WIN STATE
    # =============================
    elif game_state == "WIN":
        cv2.putText(img, f"CONGRATS! You won {current_money}", (50, 200),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 3)

        cv2.putText(img, "Show 1 finger to restart", (50, 260),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

        if fingers == 1:
            game_state = "START"

    cv2.imshow("Millionaire Vision Game", img)

    if cv2.waitKey(1) & 0xFF == 27:
        break

cap.release()
cv2.destroyAllWindows()