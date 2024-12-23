import Controls
import bluerovid


if __name__ == '__main__':
    # Create the video object
    # Add port= if is necessary to use a different one
    video = Video(port=4777)

    # Create the video object
    # Add port= if is necessary to use a different one
    model = YOLO('yolov8n.pt')
    print("Created the model")
    

    print('Initialising stream...')
    waited = 0
    while not video.frame_available():
        waited += 1
        print('\r  Frame not available (x{})'.format(waited), end='')
        cv2.waitKey(30)
    print('\nSuccess!\nStarting streaming - press "q" to quit.')

    while True:
        # Wait for the next frame to become available
        if video.frame_available():
            # Only retrieve and display a frame if it's new
            frame = video.frame()
            results = model(frame)
            annotated_frame =results[0].plot()
            cv2.imshow('frame', annotated_frame)

            #Here we wantto implement the controls for the bluerov
            




        # Allow frame to display, and check if user wants to quit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    sys.stdout.close()
    sys.stdout = sys.__stdout__


