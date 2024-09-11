import serial
import keyboard
import random


arduino = serial.Serial('COM3', 9600, timeout=1)
arduino.flushInput()
arduino.write(f"{90}\n".encode())

angle = 90

while True:
    print(f"Angle: {angle}")

    if keyboard.is_pressed('p'):
        print("pressed")
        angle += 1
    elif keyboard.is_pressed('o'):
        angle -= 1
    
    angle = random.randint(0, 180)
    arduino.write(f"{angle}\n".encode())
