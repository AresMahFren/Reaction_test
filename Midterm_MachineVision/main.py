import cv2
import mediapipe as mp
import time
import socket

# =============================
# HAND TRACKING SETUP
# =============================
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(max_num_hands=1)
mp_draw = mp.solutions.drawing_utils

UDP_IP = "127.0.0.1"
UDP_PORT = 4242
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# =============================
# QUESTIONS
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
# ROI BOX
# =============================
roi_top_left = (400, 100)
roi_bottom_right = (600, 300)

# =============================
# TIMING
# =============================
HOLD_TIME = 3
QUESTION_TIME_LIMIT = 10

hold_start_time = None
selected_fingers = None
question_start_time = time.time()

# =============================
# GAME VARIABLES
# =============================
cap = cv2.VideoCapture(0)

game_state = "START"
level = 0
current_money = "$0"

# Animation
animation_start_time = None
animation_duration = 1.5

# =============================
# FINGER COUNT FUNCTION
# =============================
def count_fingers(hand_landmarks):
    fingers = []
    tips = [4, 8, 12, 16, 20]

    if hand_landmarks.landmark[tips[0]].x < hand_landmarks.landmark[tips[0] - 1].x:
        fingers.append(1)
    else:
        fingers.append(0)

    for i in range(1, 5):
        if hand_landmarks.landmark[tips[i]].y < hand_landmarks.landmark[tips[i] - 2].y:
            fingers.append(1)
        else:
            fingers.append(0)

    return sum(fingers)

# =============================
# DRAW MONEY LADDER
# =============================
def draw_money_ladder(img, level, offset=0):
    h, w, _ = img.shape
    base_y = h - 50

    for i in range(len(money)):
        y = base_y - (i * 40) + offset

        color = (0, 140, 255)

        if i == level:
            cv2.rectangle(img, (200, y-25), (450, y+10), (0, 255, 255), -1)
            color = (0, 0, 0)

        # ⭐ SAFE LEVEL GLOW HERE
        if i == 4 or i == 9:  # $1k and $32k
            cv2.circle(img, (180, y-10), 6, (0, 255, 255), -1)

        cv2.putText(img, f"{i+1} {money[i]}", (220, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

# =============================
# MAIN LOOP
# =============================
while True:
    success, img = cap.read()
    img = cv2.flip(img, 1)

    rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    result = hands.process(rgb)

    fingers = 0
    hand_in_box = False

    # ROI
    cv2.rectangle(img, roi_top_left, roi_bottom_right, (0, 255, 0), 2)

    if result.multi_hand_landmarks:
        for handLms in result.multi_hand_landmarks:
            h, w, _ = img.shape
            x = int(handLms.landmark[9].x * w)
            y = int(handLms.landmark[9].y * h)

            if roi_top_left[0] < x < roi_bottom_right[0] and roi_top_left[1] < y < roi_bottom_right[1]:
                hand_in_box = True
                mp_draw.draw_landmarks(img, handLms, mp_hands.HAND_CONNECTIONS)
                fingers = count_fingers(handLms)

    # HOLD LOGIC
    if hand_in_box:
        if selected_fingers is None:
            selected_fingers = fingers
            hold_start_time = time.time()
        elif selected_fingers == fingers:
            elapsed = time.time() - hold_start_time
            cv2.putText(img, f"Holding: {int(elapsed)}s", (400, 350),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        else:
            selected_fingers = fingers
            hold_start_time = time.time()
    else:
        selected_fingers = None
        hold_start_time = None

    # =============================
    # START
    # =============================
    if game_state == "START":
        cv2.putText(img, "SHOW 1 FINGER TO START", (50, 200),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 3)

        if selected_fingers == 1 and hold_start_time and (time.time() - hold_start_time) >= HOLD_TIME:
            game_state = "QUESTION"
            level = 0
            current_money = "$0"
            question_start_time = time.time()
            selected_fingers = None

    # =============================
    # QUESTION
    # =============================
    elif game_state == "QUESTION":
        q, choices, correct = questions[level]

        remaining_time = QUESTION_TIME_LIMIT - int(time.time() - question_start_time)

        cv2.putText(img, f"Level {level+1} - {money[level]}", (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 0), 2)

        cv2.putText(img, f"Time: {remaining_time}s", (400, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        cv2.putText(img, q, (20, 100),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

        for i, choice in enumerate(choices):
            cv2.putText(img, f"{i+1}: {choice}", (20, 150 + i*40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 200), 2)

        if remaining_time <= 0:
            game_state = "GAME_OVER"

        if selected_fingers and hold_start_time and (time.time() - hold_start_time) >= HOLD_TIME:
            if 1 <= selected_fingers <= 4:
                if selected_fingers - 1 == correct:
                    current_money = money[level]
                    level += 1
                    game_state = "ANIMATION"
                    animation_start_time = time.time()
                    selected_fingers = None
                else:
                    game_state = "GAME_OVER"

    # =============================
    # ANIMATION
    # =============================
    elif game_state == "ANIMATION":
        elapsed = time.time() - animation_start_time
        progress = min(elapsed / animation_duration, 1)

        offset = int(progress * 40)
        draw_money_ladder(img, level-1, offset)

        cv2.putText(img, "Correct!", (200, 80),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 3)

        if progress >= 1:
            if level >= len(questions):
                game_state = "WIN"
                level = len(questions) - 1
            elif level >= 4:  # index 4 = $1,000
                game_state = "DECISION"
            else:
                game_state = "QUESTION"
                question_start_time = time.time()

    # =============================
    # DECISION
    # =============================
    elif game_state == "DECISION":
        cv2.putText(img, f"You won {current_money}", (50, 150),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 3)

        cv2.putText(img, "1 = Continue | 2 = Keep", (50, 250),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

        if selected_fingers and hold_start_time and (time.time() - hold_start_time) >= HOLD_TIME:
            if selected_fingers == 1:
                game_state = "QUESTION"
                question_start_time = time.time()
            elif selected_fingers == 2:
                game_state = "WIN"

            selected_fingers = None

    # =============================
    # GAME OVER
    # =============================
    elif game_state == "GAME_OVER":
        cv2.putText(img, "GAME OVER", (150, 200),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 0, 255), 4)

        cv2.putText(img, "1 Finger to Restart", (100, 300),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

        if selected_fingers == 1 and hold_start_time and (time.time() - hold_start_time) >= HOLD_TIME:
            game_state = "START"
            selected_fingers = None

    # =============================
    # WIN
    # =============================
    elif game_state == "WIN":
        cv2.putText(img, f"CONGRATS! {current_money}", (50, 200),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 3)

        cv2.putText(img, "1 Finger to Restart", (50, 260),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

        if selected_fingers == 1 and hold_start_time and (time.time() - hold_start_time) >= HOLD_TIME:
            game_state = "START"
            selected_fingers = None

    cv2.imshow("Millionaire Vision Game", img)

    if cv2.waitKey(1) & 0xFF == 27:
        break

    data_packet = f"{game_state},{fingers},{level},{current_money}"
    sock.sendto(data_packet.encode(), (UDP_IP, UDP_PORT))

cap.release()
cv2.destroyAllWindows()