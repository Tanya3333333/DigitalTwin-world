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





class CameraOutput_rosNode_publisher(Node):
    """ method 3: publish camera frames
    """
    def __init__(self):
        super().__init__('minimal_publisher')
        self.camera_publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self, topic, img):
        msg = String()
        msg.data = img % self.i
        self.camera_publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1
