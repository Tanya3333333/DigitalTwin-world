from ultralytics import YOLO
import numpy as np
import cv2
from companion_computer.detection.camera_detection.camera_publish import CameraOutput_mediaMTX

class YOLODetection:
    """
    follow this website for more info on YOLO via Ultralytics library: https://docs.ultralytics.com/usage/python/#key-features-of-predict-mode
    
    """

    @staticmethod
    def show_detected_frame_mediMTX(frame_queue):
        model = YOLO("yolo26n.pt")
        send_external_web = CameraOutput_mediaMTX()

        send_external_web.start_stream(1456, 1088, 30)

        while True:
            msg = frame_queue.get()   # waits until a frame arrives

            if msg is None:
                break   # stop signal

            h = msg["height"]
            w = msg["width"]

            np_img = np.frombuffer(msg["data"], dtype=np.uint8)
            np_img = np_img.reshape((h, w, 3))
            results = model.predict(source = np_img, verbose=False)

            for result in results: 
                annotated = result.plot()
                if send_external_web.ffmpeg is not None:
                    send_external_web.ffmpeg.stdin.write(annotated.tobytes())

    @staticmethod
    def show_detected_frame_opencv(frame_queue):
        model = YOLO("yolo26n.pt")
        while True:
            msg = frame_queue.get()   # waits until a frame arrives

            if msg is None:
                break   # stop signal

            h = msg["height"]
            w = msg["width"]

            np_img = np.frombuffer(msg["data"], dtype=np.uint8)
            np_img = np_img.reshape((h, w, 3))
            results = model.predict(source = np_img, verbose=False)

            for result in results: 
                annotated = result.plot()
                cv2.imshow('Interceptor View (YOLO26 Detection)', annotated)
                if cv2.waitKey(1) & 0xFF == ord("q"): break

        cv2.destroyAllWindows()
        