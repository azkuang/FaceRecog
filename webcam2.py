import cv2
import serial
import time
import numpy as np

# Initialize the serial connection to the Arduino
arduino = serial.Serial('COM3', 9600, timeout=1)
time.sleep(2)  # Wait for connection to establish
arduino.flushInput()  # Clear any leftover data in the serial input
arduino.write(f"{99}\n".encode())

# Load the deep learning model
net = cv2.dnn.readNetFromCaffe('deploy.prototxt.txt', 'res10_300x300_ssd_iter_140000.caffemodel')

def find_angle(box_left, box_right, face_center, w=640, fov=95, angle=180):
    
    pixel_per_degree = w / fov
    if face_center > box_left:
        degree_offset = (face_center - box_left) / pixel_per_degree
    elif face_center < box_right:
        degree_offset = (face_center - box_right) / pixel_per_degree
    else:
        degree_offset = 0

    return max(0, min(180, 90 + degree_offset))


cap = cv2.VideoCapture(1)

last_angle_sent = None # Start at a neutral angle
movement_threshold = 15

last_update_time = time.time()
update_interval = 0.5

while True:
    ret, frame = cap.read()
    if not ret:
        break

    (h, w) = frame.shape[:2]
    box_width = 10
    box_center = w // 2
    box_left = box_center - box_width
    box_right = box_center + box_width

    # Prepare the blob and perform a forward pass through the network
    blob = cv2.dnn.blobFromImage(cv2.resize(frame, (300, 300)), 1.0, (300, 300), (104.0, 177.0, 123.0))
    net.setInput(blob)
    detections = net.forward()

    for i in range(0, detections.shape[2]):
        confidence = detections[0, 0, i, 2]
        
        # Filter out weak detections
        if confidence > 0.5:
            # Compute the (x, y)-coordinates of the bounding box for the face
            box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
            (startX, startY, endX, endY) = box.astype("int")
            
            # Draw the bounding box of the face along with the associated probability
            cv2.rectangle(frame, (startX, startY), (endX, endY), (255, 0, 0), 2)

            # # Calculate the center of the face
            running_center = (startX + endX) // 2
            
            current_time = time.time()

            if current_time - last_update_time >= update_interval:


                # Calculate the angle to pan to center the face
                # angle = calculate_camera_pan(startX, endX, box_left, box_right, w)
                angle = find_angle(box_left, box_right, running_center)
                if angle is not None and (last_angle_sent is None or abs(angle - last_angle_sent) > movement_threshold):
                    print(f"Box Center: {box_center}")      
                    print(f"Face Center: {running_center}")
                    print(f"Angle: {angle}")
                    arduino.write(f"{angle}\n".encode())
                    last_angle_sent = angle
                    last_update_time = current_time

    cv2.imshow('Frame', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Cleanup
cap.release()
cv2.destroyAllWindows()
