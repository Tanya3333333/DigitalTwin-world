from ultralytics import YOLO
import cv2
import numpy as np

class YOLOTrack:
    """
    follow this website for more info on YOLO via Ultralytics library: https://docs.ultralytics.com/modes/track/#persisting-tracks-loop
    
    """

    @staticmethod
    def show_tracked_frame_opencv(frame_queue):

        # Load the YOLO26 model
        model = YOLO("yolo26n.pt")
        model.to("cuda")

        # Loop through the video frames
        while frame_queue is not None:

            msg = frame_queue.get()   # waits until a frame arrives

            h = msg["height"]
            w = msg["width"]
            np_img = np.frombuffer(msg["data"], dtype=np.uint8) #take raw bytes and view them as NumPy 1D array
            np_img = np_img.reshape((h, w, 3)) # makes its a 3D array (height, width, channels = BRG (so 3!))
             
            # Run YOLO26 tracking on the frame, persisting tracks between frames            
            results = model.track(np_img, persist=True, conf=0.5, iou=0.7, tracker="bytetrack.yaml")  # with ByteTrack
            

            # Visualize the results on the frame
            annotated_frame = results[0].plot()

            # Display the annotated frame
            cv2.imshow("Interceptor View (YOLO26 Tracking)", annotated_frame)

            if cv2.waitKey(1) & 0xFF == ord("q"): break
                
        cv2.destroyAllWindows()