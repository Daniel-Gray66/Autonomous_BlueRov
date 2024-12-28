if __name__ == '__main__':
    # Comment this out so we can see print statements:
    # sys.stdout = open(os.devnull, 'w')  

    # 1. Load YOLO model
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

    # 5. Wait for frames
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
                continue  # or break, depending on your needs

            results = model(frame)
            annotated_frame = results[0].plot()

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
                control_motors(master, 1500, 1500, 1500, 1600, 1500, 1500)
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
