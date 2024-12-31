
#!/usr/bin/env python
"""
BlueRov video capture class
"""

import cv2
import numpy as np
import gi
from ultralytics import YOLO
import os 
import sys

from Video_class.py import Video

from testingv2 import (
    create_connectionlink,
    arm_vehicle,
    disarm_vehicle,
    control_motors
)

if __name__ == '__main__':
    
    #These are all set up to ensure that the robot is ready for operation

    # 1. Load YOLO model (will need to be the custome data-set eventually)
    model = YOLO('yolov8n.pt')
    print("Created the YOLO model")

    # 2. Set up video
    video = Video()
    print("Video object created.")

    # 3. Connect to ROV
    master = create_connectionlink()
    print("Connection created.")
    if not master:
        sys.exit("Failed to establish MAVLink connection. Exiting.")

    # 4. Arm vehicle
    if not arm_vehicle(master):
        sys.exit("Vehicle refused to arm. Exiting.")
    print("Vehicle armed.")


    # 5. Wait for frames might turn this in to a different function idk 
    print('Initialising stream...')
    waited = 0
    while not video.frame_available():
        waited += 1
        print(f'\r  Frame not available (x{waited})', end='')
        cv2.waitKey(30)
    print('\nSuccess!\nStarting streaming - press "q" to quit.')

    while True:
        if video.frame_available():
            frame = video.frame()
            if frame is None:
                continue  

            results = model(frame)
            annotated_frame = results[0].plot()

            #Here we want the robot to be connstantly searching



            # Debug: see if YOLO is detecting anything
            boxes = results[0].boxes
            detected_person = False
            for box in boxes:
                class_id = int(box.cls[0])
                class_name = results[0].names[class_id]
                print(f"Detected: {class_name}")
                if class_name == "person":
                    detected_person = True
                    break

            # Motor control
            if detected_person:
                print("Person detected! Moving forward...")
                # Make sure to pass correct number of motor values:
                control_motors(master, 1500, 1500, 1500, 1600, 1600, 1500)
                
                #here we need to add more logic to follow the objevt
                #We will need to update our moving parameters above to follow the robot


            else:
                # No person => neutral
                control_motors(master, 1500, 1500, 1500, 1500, 1500, 1500)

            cv2.imshow('Fishy device', annotated_frame)

        # Quit on 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            # Stop motors
            control_motors(master, 1500, 1500, 1500, 1500, 1500, 1500)
            disarm_vehicle(master)
            break

    cv2.destroyAllWindows()
    print("Exiting program.")