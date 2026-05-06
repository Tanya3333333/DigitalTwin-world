import torch
print(torch.cuda.is_available())

import asyncio, time, math, cv2, os, csv
import numpy as np
import multiprocessing as mp 

from projectairsim import ProjectAirSimClient, World, Drone
from projectairsim.utils import projectairsim_log 


from kernel.interceptor.main_uav_state import InterceptorStateServer
from kernel.other_uavs.target_drone_trajectory import TargetDroneTrajectory

#from companion_computer.detection.camera_detection.camera_publish import CameraOutput_rosNode_publisher
#from companion_computer.detection.camera_detection.yolo_detection import YOLODetection
from companion_computer.track.camera_track.yolo_track import YOLOTrack


# log 
desktop = os.path.expanduser("~/Desktop") 
CAMERA_LOG_PATH = os.path.join(desktop, "timestamp_camera_frame_publish_log.csv")
INTERCEPTOR_LOG_PATH = os.path.join(desktop, "timestamp_interceptor_pose_log.csv")
TARGET_DRONE_LOG_PATH = os.path.join(desktop, "timestamp_target_drone_pose_log.csv")
SIM_EXE_LOG_PATH = os.path.join(desktop, "timestamp_sim_execution_log.csv")

# sim stepping rate 
INTERCEPTOR_RATE = 100
TARGET_DRONE_RATE = 100
PA_RATE = 100 # project airsim rate
PA_DT = 1 / PA_RATE

class VisualizerKernel:
    def __init__(self):
        #initalize the connection with the unreal 
        self.PAC = ProjectAirSimClient()
        self.PAC.connect() 

        #sim stepping synchronization
        self.cycle = 0
        self.interceptor_update_t_log = []
        self.targetDrone_update_t_log = []
        self.sime_exec_t_log = []


        if(math.isclose(INTERCEPTOR_RATE/PA_RATE, 1)):self.sync_interceptor_to_PA = 1, print ("Interceptor state step is synchronized to Project Airsim sim step")
        else: self.sync_interceptor_to_PA = 0

        if(math.isclose(TARGET_DRONE_RATE/PA_RATE, 1)):self.sync_targetDrone_to_PA = 1, print ("Target drone state step is synchronized to Project Airsim sim step")
        else: self.sync_targetDrone_to_PA = 0


        # initialize world and drones
        self.world = World(self.PAC, "scene_cesium_drone.jsonc", delay_after_load_sec=2)

        self.interceptor = Drone(self.PAC, self.world, "interceptor")
        self.interceptor_maneuver = InterceptorStateServer()

        self.targetDrone = Drone(self.PAC, self.world, "target")
        self.targetDrone_maneuver = TargetDroneTrajectory()

        self.targetDrone2 = Drone(self.PAC, self.world, "target2")
        self.targetDrone2_maneuver = TargetDroneTrajectory()

        self.targetDrone3 = Drone(self.PAC, self.world, "target3")
        self.targetDrone3_maneuver = TargetDroneTrajectory()

        # camera related
        self.frame_queue = mp.Queue(maxsize=5)
        self.camera_topic = '/Sim/SceneBasicDrone/robots/interceptor/sensors/InterceptionCamera/scene_camera'
        self.camera_frame_time_of_arrival = [] #store the timestamp of arrival of each frame


    def precise_sleep (self, seconds):
        start_time = time.perf_counter()
        while time.perf_counter() - start_time < seconds:
            pass



    def update_target_drones(self):
        pose = self.targetDrone_maneuver.yz_circle_trajectory()
        pose2 = self.targetDrone2_maneuver.vertical_occilation_trajectory()
        pose3 = self.targetDrone3_maneuver.lawnmower_trajectory()

        pose2["translation"]["y"] += 10
        pose3["translation"]["y"] -= 10

        self.targetDrone.set_pose(pose)
        self.targetDrone2.set_pose(pose2)
        self.targetDrone3.set_pose(pose3)
    

    def step_sim (self):

        # update interceptor pose
        if(self.sync_interceptor_to_PA):
            self.interceptor_update_t_log.append(time.perf_counter())
            interceptor_latest_pose = self.interceptor_maneuver._states()
            self.interceptor.set_pose(interceptor_latest_pose)

        elif(self.cycle % (PA_RATE // INTERCEPTOR_RATE) == 0):
            self.interceptor_update_t_log.append(time.perf_counter())
            interceptor_latest_pose = self.interceptor_maneuver._states()
            self.interceptor.set_pose(interceptor_latest_pose)


        # update target drone pose
        if(self.sync_targetDrone_to_PA):
            self.targetDrone_update_t_log.append(time.perf_counter())
            self.update_target_drones()

        elif(self.cycle % (PA_RATE // TARGET_DRONE_RATE) == 0):
            self.targetDrone_update_t_log.append(time.perf_counter())
            self.update_target_drones()


    
    def camera_frame_callback(self, topic, msg):

        if msg is None: return
        
        self.camera_frame_time_of_arrival.append(msg["time_stamp"])
            
        if self.frame_queue.full():             # Keep only recent frames, do not let queue grow forever
            try: self.frame_queue.get_nowait()   # remove oldest frame
            except: pass

        self.frame_queue.put(msg)


    def manager(self): 

        # Shows the topic that can be played around with
        self.PAC.get_topic_info() 
        
        
        self.PAC.subscribe(self.camera_topic, self.camera_frame_callback)       

        #detection_process = mp.Process(target=YOLODetection().show_detected_frame_opencv, args=(self.frame_queue,))
        track_process = mp.Process(target=YOLOTrack.show_tracked_frame_opencv, args=(self.frame_queue,))
        track_process.start()
        
        next_t = time.perf_counter()

        try: 
            print ("starting drone(s) manuvering...")
            while True: 
                now = time.perf_counter()
                
                # wait to reach the expected sim rate
                if now < next_t: 
                    self.precise_sleep(next_t - now)
                
                start = time.perf_counter()
                self.step_sim()
                end = time.perf_counter()

                # check timing of the loop = it should be ~ 0.01 s
                sim_exec_t = end - start
                lateness = end - next_t
                self.sime_exec_t_log.append(start)

                # if the loop step was faster than expected sim time 
                if lateness > PA_DT: 
                    print(
                        f"Overrun: sim execution time = {sim_exec_t:.2} s     "
                        f"late by {lateness:.2} s"
                    )
                self.cycle +=1
                next_t += PA_DT

        except KeyboardInterrupt:

            print("End the visualizer..")

            # ai models
            self.frame_queue.put(None)
            track_process.join()
            
            self.writeListToCsv(self.camera_frame_time_of_arrival, "camera_frame_publish_timestamp", CAMERA_LOG_PATH)
            self.writeListToCsv(self.interceptor_update_t_log, "interceptor_pose_timestamp", INTERCEPTOR_LOG_PATH)
            self.writeListToCsv(self.targetDrone_update_t_log, "targetDrone_pose_timestamp", TARGET_DRONE_LOG_PATH)
            self.writeListToCsv(self.sime_exec_t_log, "sim_execution_timestamp", SIM_EXE_LOG_PATH)
            

            self.PAC.disconnect()   # stop interaction with Project Airsim api server


    def writeListToCsv(self, dataList, header, fileDir):
        """ 
        log_list in [] 
        header is in str
        fileDir = LOG_PATH """

        with open(fileDir, 'w', newline='') as file: 
            writer = csv.writer(file) 
            writer.writerow([header]) 
            for val in dataList: 
                writer.writerow([val]) 
            file.close()
        print(f"Successfully wrote data to {fileDir}")


if __name__ == "__main__":
    interceptor_interface = VisualizerKernel()
    interceptor_interface.manager()