import cv2
import os
import serial
import time

arduino = serial.Serial('COM3', 9600, timeout=1)
time.sleep(2)

arduino.flushInput()

arduino.write(f"90\n".encode())

def map_value(center, running_center, screen_width, angle):
    return ((center - running_center) * center) / angle

def calculate_camera_pan(face_center_x, box_left, box_right, screen_width, max_servo_angle):
    if face_center_x < box_left:
        # Face is to the left of the box, calculate how much to pan left
        pixel_difference = face_center_x - box_left
    elif face_center_x > box_right:
        # Face is to the right of the box, calculate how much to pan right
        pixel_difference = face_center_x - box_right
    else:
        # Face is within the box, no need to pan
        return 0

    # Convert pixel difference to angle difference
    angle_difference = (pixel_difference / screen_width) * max_servo_angle
    
    return angle_difference


cap = cv2.VideoCapture(1)
face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

# running_center = None
last_angle_sent = 0  
movement_threshold = 4

while True:
    ret, frame = cap.read()
    if not ret:
        break

    screen_w = frame.shape[1]
    screen_h = frame.shape[0]

    box_w = 100
    box_left = (screen_w - box_w) // 2
    box_right = box_left + box_w
    
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    faces = face_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))

    for (x, y, w, h) in faces:
        cv2.rectangle(frame, (x, y), (x+w, y+h), (255, 0, 0), 2)
        face_roi = gray[y:y+h, x:x+w]

        center = frame.shape[1] // 2

        running_center = x + w // 2

        # print(f"Center: {center}")

        # Map the face center to a servo angle
        # angle = map_value(center, running_center, frame.shape[1], 180)
        angle = calculate_camera_pan(running_center, box_left, box_right, screen_w, 180)
        print(f"Angle: {angle}")
        
        if last_angle_sent is None or abs(angle - last_angle_sent) > movement_threshold:
            arduino.write(f"{angle}\n".encode())
            last_angle_sent = angle

    cv2.imshow('frame', frame)
    
    if cv2.waitKey(1) == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()