import numpy as np
import time
import cv2 # good website to study the lib: https://opencv.org/read-display-and-write-an-image-using-opencv/ 
import subprocess # this library lets python run external programs such as ffmpeg

class CameraOutput_opencv:

    """ method 1: This callback function can have 3 diferent ways of manipultating the camera output.
         1) Using rerun library to generate camera frames in real time
          2) Using CV2 (OpenCV) library to show the pop-up window for all the frames
        """

    def show_camera_result(self, topic, msg):
        
        if msg is None: return

        h = msg["height"]
        w = msg["width"]
        np_img = np.frombuffer(msg["data"], dtype=np.uint8) #take raw bytes and view them as NumPy 1D array
        np_img = np_img.reshape((h, w, 3)) # makes its a 3D array (height, width, channels = BRG (so 3!))

        # Display the image in a pop-up window
        cv2.imshow('Intercepto View', np_img)
        cv2.waitKey(1) # wait 1 ms to assess if key pressed, otherwise continue


class CameraOutput_mediaMTX:
    """ method 2: In order to use this function, make sure to install the mediaMTX and ffmpeg executable files to be able to see the camera outputs in an external web browser. 
        1) install the mediaMTX from the asset list according to the current OS in use: https://github.com/bluenviron/mediamtx/releases 
        2) then in your OS terminal, go to the path of your download and run the mediamtx.exe file
        3) install the ffmpeg: https://ffmpeg.org/download.html and make sure its added to PATH (System Environment Variables)
        """
    def __init__(self):
        self.ffmpeg = None

    
    def start_stream(self, width, height, fps):
        cmd = [
            r"E:\tanya_download_stuffs\ffmpeg-8.1-essentials_build\ffmpeg-8.1-essentials_build\bin\ffmpeg.exe",
            "-f", "rawvideo",
            "-pix_fmt", "bgr24",
            "-s", f"{width}x{height}",
            "-r", str(fps),
            "-i", "-",
            "-an",
            "-c:v", "libx264",
            "-preset", "ultrafast",
            "-tune", "zerolatency",
            "-pix_fmt", "yuv420p",
            "-f", "rtsp",
            "-rtsp_transport", "tcp",
            "rtsp://127.0.0.1:8554/interceptor"
        ]

        self.ffmpeg = subprocess.Popen(cmd, stdin=subprocess.PIPE) 


    def show_camera_result(self, topic, msg):

        if msg is None: return

        h = msg["height"]
        w = msg["width"]
        np_img = np.frombuffer(msg["data"], dtype=np.uint8) #take raw bytes and view them as NumPy 1D array

        if self.ffmpeg is not None:
            self.ffmpeg.stdin.write(np_img.tobytes())


    def stop_stream(self):
        if self.ffmpeg is not None:
            if self.ffmpeg.stdin:
                self.ffmpeg.stdin.close()
            self.ffmpeg.terminate()
            self.ffmpeg.wait()
            self.ffmpeg = None



### multi-processing methods


class opencv_mp:
    """
    gets the frames from the shared queue (from camera callback) and show it as a pop-up window
    """

    @staticmethod
    def show_frame_opencv(frame_queue):

        # Loop through the video frames
        while True:
            msg = frame_queue.get()

            if msg is None:
                break

            h = msg["height"]
            w = msg["width"]
            np_img = np.frombuffer(msg["data"], dtype=np.uint8) #take raw bytes and view them as NumPy 1D array
            np_img = np_img.reshape((h, w, 3)) # makes its a 3D array (height, width, channels = BRG (so 3!))

            # Display the annotated frame
            cv2.imshow("Interceptor Camera View", np_img)

            if cv2.waitKey(1) & 0xFF == ord("q"): break
                
        cv2.destroyAllWindows()



class opencv_vid_mp:
    """
    gets the frames from the shared queue (from camera callback) and show it as the pop-up window + save the video of the sim at the end
    """

    @staticmethod
    def show_frame_opencv(frame_queue):
        writer = None
        fps = 30
        output_path = "interceptor_camera_recording.mp4"

        try:
            while True:
                msg = frame_queue.get()

                if msg is None:
                    break

                h = msg["height"]
                w = msg["width"]

                np_img = np.frombuffer(msg["data"], dtype=np.uint8)
                np_img = np_img.reshape((h, w, 3))

                if writer is None:
                    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
                    writer = cv2.VideoWriter(output_path, fourcc,fps,(w, h))

                writer.write(np_img)

                cv2.imshow("Interceptor Camera View", np_img)

                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break

        except KeyboardInterrupt:
            pass

        finally:
            if writer is not None:
                writer.release()

            cv2.destroyAllWindows()
