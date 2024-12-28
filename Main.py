import cv2
import sys
import time
from ultralytics import YOLO


from testingv2 import (
    create_connectionlink,
    arm_vehicle,
    disarm_vehicle,
    control_motors
)
from bluerov_video_stream import(
    start_gst,
    gst_to_opencv,
    frame,
    frame_available,
    run,
    callback,



)

# Suppose you have a Video class for capturing frames
# from some_camera_module import Video

def main():
    # 1. Initialize camera and YOLO model
    video = Video(port=4777)
    model = YOLO('yolov8n.pt')  # or your custom model that can detect 'bottle'
    print("Model loaded.")

    # Wait for the camera stream
    print('Initializing stream...')
    waited = 0
    while not video.frame_available():
        waited += 1
        print(f'\r  Frame not available (x{waited})', end='')
        cv2.waitKey(30)
    print('\nSuccess! Camera stream is ready.')

    # 2. Connect to ROV
    master = create_connectionlink()
    if not master:
        sys.exit("Failed to establish MAVLink connection. Exiting.")

    # 3. Arm vehicle
    if not arm_vehicle(master):
        sys.exit("Vehicle refused to arm. Exiting.")

    print("Starting YOLO detection. Press 'q' to quit...")

    try:
        while True:
            if video.frame_available():
                frame = video.frame()

                # Run YOLO inference; optionally set confidence (e.g., conf=0.3)
                results = model(frame)
                annotated_frame = results[0].plot()

                # ------------------------------------------------------
                # DETECTION LOGIC: Check if we see a 'bottle'
                # ------------------------------------------------------
                detected_bottle = False

                # results[0].boxes has the bounding boxes
                boxes = results[0].boxes

                # Each box has .cls (class index), .conf (confidence), etc.
                # results[0].names is a dict {class_index: class_name}
                for box in boxes:
                    class_id = int(box.cls[0])
                    class_name = results[0].names[class_id]
                    # If the model's class name is 'bottle'
                    if class_name == "bottle":
                        detected_bottle = True
                        break  # We only need one bottle to move

                # ------------------------------------------------------
                # MOTOR CONTROL: Move only if a bottle is detected
                # ------------------------------------------------------
                if detected_bottle:
                    print("Bottle detected! Moving forward...")
                    control_motors(
                        master,
                        roll=1500,
                        pitch=1600,   # slight forward
                        throttle=1500,
                        yaw=1500
                    )
                else:
                    # No bottle => stay neutral
                    control_motors(
                        master,
                        roll=1500,
                        pitch=1500,
                        throttle=1500,
                        yaw=1500
                    )

                # Show the annotated frame
                cv2.imshow('ROV Camera', annotated_frame)

            # Check for quit key
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            # Small delay so we don't max out CPU
            time.sleep(0.01)

    except KeyboardInterrupt:
        print("KeyboardInterrupt received. Exiting...")

    finally:
        # Stop motors
        control_motors(
            master,
            roll=1500,
            pitch=1500,
            throttle=1500,
            yaw=1500
        )
        disarm_vehicle(master)
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()