import cv2
import mediapipe as mp
import socket

# --- CONNECTION ---
UDP_IP = "127.0.0.1"
UDP_PORT = 4242 
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# --- AI SETUP ---
mp_hands = mp.solutions.hands
# Increased confidence levels for "locking" onto the hand
hands = mp_hands.Hands(max_num_hands=1, min_detection_confidence=0.85, min_tracking_confidence=0.85)
mp_draw = mp.solutions.drawing_utils

cap = cv2.VideoCapture(0)

while cap.isOpened():
    success, image = cap.read()
    if not success: break

    image = cv2.flip(image, 1)
    rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    results = hands.process(rgb_image)

    gesture_state = "IDLE"
    final_selection = 0
    
    if results.multi_hand_landmarks:
        for hand_landmarks in results.multi_hand_landmarks:
            mp_draw.draw_landmarks(image, hand_landmarks, mp_hands.HAND_CONNECTIONS)
            lm = hand_landmarks.landmark

            # 1. Finger Up Checks
            index_up  = lm[8].y < lm[6].y
            middle_up = lm[12].y < lm[10].y
            ring_up   = lm[16].y < lm[14].y
            pinky_up  = lm[20].y < lm[18].y
            # Thumb: Is the tip significantly higher than the rest of the hand?
            thumb_up  = lm[4].y < lm[3].y and lm[4].y < lm[5].y

            # 2. Gesture Logic
            # THUMBS UP (Restart to Menu)
            if thumb_up and not index_up and not middle_up and not ring_up and not pinky_up:
                gesture_state = "THUMBS_UP"
                final_selection = 9 # Special code for Thumbs Up
            
            # FIST (Restart Question 1)
            elif not index_up and not middle_up and not ring_up and not pinky_up and not thumb_up:
                gesture_state = "FIST"
                final_selection = 0 # 0 fingers = Fist
            
            # SELECTIONS 1-4
            elif index_up and not middle_up and not ring_up and not pinky_up:
                final_selection = 1
            elif index_up and middle_up and not ring_up and not pinky_up:
                final_selection = 2
            elif index_up and middle_up and ring_up and not pinky_up:
                final_selection = 3
            elif index_up and middle_up and ring_up and pinky_up:
                final_selection = 4

    # --- SEND TO GODOT ---
    # We send gesture_state so Godot knows exactly what gesture is happening
    message = f"{gesture_state},{final_selection},0,$0" 
    sock.sendto(message.encode(), (UDP_IP, UDP_PORT))

    # UI Feedback
    cv2.putText(image, f"STATE: {gesture_state} | VAL: {final_selection}", (50, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
    cv2.imshow('Millionaire Vision', image)
    if cv2.waitKey(1) & 0xFF == ord('q'): break
    
cap.release()
cv2.destroyAllWindows()