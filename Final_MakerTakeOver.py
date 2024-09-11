import cv2
import serial
import time
import numpy as np

# Initialize the serial connection to the Arduino
arduino = serial.Serial('COM3', 9600, timeout=1)
time.sleep(2)  # Wait for connection to establish
arduino.flushInput()  # Clear any leftover data in the serial input

angle = 90
arduino.write(f"{angle}\n".encode())

last_update_time = time.time()
update_interval = 5

last_angle_sent = None
threshold = 25

largest_area = 0
largest_face = None

# Load the deep learning model
net = cv2.dnn.readNetFromCaffe('deploy.prototxt.txt', 'res10_300x300_ssd_iter_140000.caffemodel')

cap = cv2.VideoCapture(1)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    (h, w) = frame.shape[:2]

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

            # area = (endX - startX) * (endY - startY)

            # if area > largest_area:
            #     largest_area = area
            #     largest_face = (startX, startY, endX, endY)

            current_time = time.time()

            # Multiple faces
            # if largest_face is not None:
            #     (startX, startY, endX, endY) = largest_face
                
            #     cv2.rectangle(frame, (startX, startY), (endX, endY), (255, 0, 0), 2)

            if current_time - last_update_time >= update_interval or abs(last_angle_sent - angle) > threshold:

                center = w / 2
                center_face = (startX + endX) / 2

                center_diff = center_face - center
                curr_diff = center_diff

                print(f'center_diff: {center_diff}')

                # if center_diff > 0:
                #     angle += 5
                # elif center_diff < 0:
                #     angle -= 5

                pixel_per_degree = w / 95
                degree_offset = center_diff / pixel_per_degree

                angle = max(0, min(180, 90 + degree_offset))

                print(180 - angle)

                # face = '{0:d}'.format((startX+endX) // 2)
                
                arduino.write(f'{180 - angle}\n'.encode())

                last_angle_sent = angle

                last_update_time = current_time

                # if center_diff == curr_diff:
                #     arduino.write(f'{90}\n'.encode())

            cv2.rectangle(frame, (startX, startY), (endX, endY), (255, 0, 0), 2)

    cv2.imshow('Frame', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Cleanup
cap.release()
cv2.destroyAllWindows()
