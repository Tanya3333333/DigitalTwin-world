from ultralytics import YOLO
import cv2
import numpy as np
from companion_computer.yolo_models import train_yolo11n_960_rgb_all_trainval_gpu0

class YOLOTrack:
    """
    follow this website for more info on YOLO via Ultralytics library: https://docs.ultralytics.com/modes/track/#persisting-tracks-loop
    
    """

    @staticmethod
    def show_tracked_frame_opencv(frame_queue):

        # Load the YOLO26 model
        model = YOLO(train_yolo11n_960_rgb_all_trainval_gpu0)
        model.to("cuda")

        # Loop through the video frames
        while True:
            msg = frame_queue.get()

            if msg is None:
                break

            h = msg["height"]
            w = msg["width"]
            np_img = np.frombuffer(msg["data"], dtype=np.uint8) #take raw bytes and view them as NumPy 1D array
            np_img = np_img.reshape((h, w, 3)) # makes its a 3D array (height, width, channels = BRG (so 3!))
             
            # Run YOLO26 tracking on the frame, persisting tracks between frames            
            results = model.track(np_img, persist=True, tracker="bytetrack.yaml")  # with ByteTrack
            

            # Visualize the results on the frame
            annotated_frame = results[0].plot()

            # Display the annotated frame
            cv2.imshow("Interceptor View (YOLO26 Tracking)", annotated_frame)

            if cv2.waitKey(1) & 0xFF == ord("q"): break
                
        cv2.destroyAllWindows()