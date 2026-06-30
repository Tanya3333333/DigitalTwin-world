import torch
print(torch.cuda.is_available())

import os, csv, time, ctypes, asyncio
import multiprocessing as mp

from projectairsim import ProjectAirSimClient, World, Drone

from kernel.interceptor.main_uav_state_server import interceptor_udp_receiver
from kernel.other_uavs.target_drone_trajectory_custom import TargetDroneTrajectory
from kernel.other_uavs.target_drone_trajectory_PA_api import PATargetDroneTrajectory

from companion_computer.detection_and_track.camera_track.yolo_track import YOLOTrack
from kernel.interceptor.camera_publish_callback import opencv_mp, opencv_vid_mp


# log paths
desktop = os.path.expanduser("~/Desktop")

RT_RATIO = os.path.join(desktop, "realtime_ratio.csv")
LOOP_LOG_PATH = os.path.join(desktop, "timestamp_loop_log.csv")
PA_LOOP_LOG_PATH = os.path.join(desktop, "timestamp_project_airsim_loop_log.csv")
STEP_LOG_PATH = os.path.join(desktop, "airsim_step_time.csv")
INTERCEPTOR_LOG_PATH = os.path.join(desktop, "pose_computation_time.csv")
TARGET_DRONE_LOG_PATH = os.path.join(desktop, "set_pose.csv")
WALLCLK_CAMERA_LOG_PATH = os.path.join(desktop, "wall_dt_camera.csv")
PA_CAMERA_LOG_PATH = os.path.join(desktop, "PA_dt_camera.csv")

# Constants for testing purposes (otherwise the dt step of sim can be read from the sim_config automatically)
EXPECTED_DT = 0.01


class VisualizerKernel:
    def __init__(self):

        # increase resolution of the determinsitic reaction to time related variables
        try:
            ctypes.windll.winmm.timeBeginPeriod(1)
            print("[Timer] Windows timer resolution set to 1 ms")
        except Exception as e:
            print("[Timer] Could not set timer resolution:", e)

        # initialize connection with Unreal / Project AirSim
        self.PAC = ProjectAirSimClient()
        self.PAC.connect()

        # timing analysis lists
        self.realtime_ratio_log = []
        self.loop_dt_log = []
        self.project_airsim_loop_dt_log = []
        self.airsim_step_time_log = []
        self.pose_computation_time = []
        self.set_pose = []
        self.wallClk_camera_frame_arrival_dt = []
        self.PA_camera_frame_arrival_dt = []
        self.last_wall_ts = None
        self.last_sim_ts = None

        # initialize world and drones
        self.world = World(self.PAC, "scene_for_drones.jsonc", delay_after_load_sec=2)

        # get step-ns from config
        self.expected_dt = self.world.sim_config["clock"]["step-ns"]


        self.interceptor = Drone(self.PAC, self.world, "interceptor")
        self.targetDrone1 = Drone(self.PAC, self.world, "target1")
        self.targetDrone2 = Drone(self.PAC, self.world, "target2")
        self.targetDrone3 = Drone(self.PAC, self.world, "target3")

        self.manager_mp = mp.Manager()
        self.shared_state_interceptor = self.manager_mp.dict()
        self.shared_state_interceptor["pose"] = {
            "frame_id": "DEFAULT_FRAME",
            "translation": {"x": 0, "y": 0, "z": 0},
            "rotation": {"w": 1, "x": 0, "y": 0, "z": 0},
        }

        self.latest_pose_target_uav1 = {
            "frame_id": "DEFAULT_FRAME",
            "translation": {"x": 0, "y": 0, "z": 0},
            "rotation": {"w": 1, "x": 0, "y": 0, "z": 0},
        }

        self.latest_pose_target_uav2 = {
            "frame_id": "DEFAULT_FRAME",
            "translation": {"x": 0, "y": 0, "z": 0},
            "rotation": {"w": 1, "x": 0, "y": 0, "z": 0},
        }

        self.latest_pose_target_uav3 = {
            "frame_id": "DEFAULT_FRAME",
            "translation": {"x": 0, "y": 0, "z": 0},
            "rotation": {"w": 1, "x": 0, "y": 0, "z": 0},
        }

        self.latest_target_poses = [
            self.latest_pose_target_uav1,
            self.latest_pose_target_uav2,
            self.latest_pose_target_uav3,
        ]

        self.udp_stop_event = mp.Event()
        self.udp_process = mp.Process(
            target=interceptor_udp_receiver,
            args=(self.shared_state_interceptor, self.udp_stop_event)
        )

        # target drone trajectory objects
        self.target_uav1_maneuver = TargetDroneTrajectory()
        self.target_uav2_maneuver = TargetDroneTrajectory()
        self.target_uav3_maneuver = TargetDroneTrajectory()

        # camera topic
        self.camera_topic = "/Sim/SceneBasicDrone/robots/interceptor/sensors/InterceptionCamera/scene_camera"

        # shows the scene
        self.frame_queue = mp.Queue(maxsize=5)
        self.camera_frame_process = mp.Process(target=opencv_vid_mp.show_frame_opencv,args=(self.frame_queue,))
        #self.track_process = mp.Process(target=YOLOTrack.show_tracked_frame_opencv, args=(self.frame_queue,))




### helpers 
    def writeListToCsv(self, dataList, header, fileDir):
        with open(fileDir, "w", newline="") as file:
            writer = csv.writer(file)
            writer.writerow([header])

            for val in dataList:
                writer.writerow([val])

        print(f"Successfully wrote data to {fileDir}")       

    def camera_frame_callback(self, topic, msg):
        if msg is None:
            return
        
        now_wall = time.perf_counter_ns()
        sim_ts = msg["time_stamp"]
        
        if self.frame_queue.full():             # Keep only recent frames, do not let queue grow forever
            try: self.frame_queue.get_nowait()   # remove oldest frame
            except: pass
        self.frame_queue.put(msg)
        
        # timing analysis
        if self.last_wall_ts is not None and self.last_sim_ts is not None:
            camera_wall_dt = (now_wall - self.last_wall_ts) / 1e9
            camera_sim_dt = (sim_ts - self.last_sim_ts) / 1e9
            self.wallClk_camera_frame_arrival_dt.append(camera_wall_dt)
            self.PA_camera_frame_arrival_dt.append(camera_sim_dt)

        self.last_wall_ts = now_wall
        self.last_sim_ts = sim_ts
        
        

    def compute_next_target_uav_poses(self, dt_ns):
        dt_s = dt_ns * 1e-9

        pose1 = self.target_uav1_maneuver.vertical_occilation_trajectory(dt_s)
        pose2 = self.target_uav2_maneuver.takeOff_landing_trajectory(dt_s, 40)
        pose3 = self.target_uav3_maneuver.lawnmower_trajectory(dt_s)

        pose2["translation"]["y"] += 5
        pose3["translation"]["y"] -= 5

        return [pose1, pose2, pose3]

    def step_drones(self):
        self.interceptor.set_pose(self.shared_state_interceptor["pose"])
        self.targetDrone1.set_pose(self.latest_target_poses[0])
        self.targetDrone2.set_pose(self.latest_target_poses[1])
        self.targetDrone3.set_pose(self.latest_target_poses[2])

### main function
    def manager(self):

        self.udp_process.start()
        self.PAC.subscribe(self.camera_topic, self.camera_frame_callback)
        self.camera_frame_process.start()
        #self.track_process.start()
        
        try:
            while True:
                full_start = time.perf_counter_ns()
                PA_full_start = self.world.get_sim_time()

                step_start = time.perf_counter_ns()
                self.world.continue_for_single_step(wait_until_complete=True)
                step_end = time.perf_counter_ns()

                setpose_start = time.perf_counter_ns()
                self.step_drones()
                setpose_end = time.perf_counter_ns()

                compute_start = time.perf_counter_ns()
                self.latest_target_poses = self.compute_next_target_uav_poses(self.expected_dt)
                compute_end = time.perf_counter_ns()

                

                PA_full_end = self.world.get_sim_time()
                full_end = time.perf_counter_ns()


                # timing evaluations
                wallClk_full_loop_dt = (full_end - full_start) / 1e9
                project_airsim_full_loop_dt = (PA_full_end - PA_full_start)
                walkClk_airsim_step_dt = (step_end - step_start) / 1e9
                setpose_dt = (setpose_end - setpose_start) / 1e9
                compute_next_pose_dt = (compute_end - compute_start) / 1e9

                wall_dt = full_end - full_start
                realtime_ratio = self.expected_dt / wall_dt

                self.loop_dt_log.append(wallClk_full_loop_dt)
                self.project_airsim_loop_dt_log.append(project_airsim_full_loop_dt)
                self.airsim_step_time_log.append(walkClk_airsim_step_dt)
                self.set_pose.append(setpose_dt)
                self.pose_computation_time.append(compute_next_pose_dt)
                self.realtime_ratio_log.append(realtime_ratio)

        except KeyboardInterrupt: 
            print("Stopping sim...")


            # kill all processes

            self.udp_stop_event.set()

            try:
                while self.frame_queue.full():
                    self.frame_queue.get_nowait()
                self.frame_queue.put_nowait(None)
            except:pass


            if self.camera_frame_process.is_alive():
                self.camera_frame_process.join(timeout=2)
                if self.camera_frame_process.is_alive():
                    self.camera_frame_process.terminate()
                    self.camera_frame_process.join()

            if self.udp_process.is_alive():
                self.udp_process.join(timeout=2)
                if self.udp_process.is_alive():
                    self.udp_process.terminate()
                    self.udp_process.join()

            

            #if self.track_process.is_alive():
                #self.track_process.terminate()
            
            # write logs 
            self.writeListToCsv(self.realtime_ratio_log, "realtime_ratio", RT_RATIO)
            self.writeListToCsv(self.loop_dt_log, "full_loop_dt", LOOP_LOG_PATH)
            self.writeListToCsv(self.project_airsim_loop_dt_log , "PA_full_loop_dt" , PA_LOOP_LOG_PATH)
            self.writeListToCsv(self.airsim_step_time_log, "airsim_step_time", STEP_LOG_PATH)
            self.writeListToCsv(self.pose_computation_time, "pose_computation_time", INTERCEPTOR_LOG_PATH)
            self.writeListToCsv(self.set_pose, "set_pose", TARGET_DRONE_LOG_PATH)
            self.writeListToCsv(self.wallClk_camera_frame_arrival_dt, "wallClk_camera_frame_publish_timestamp", WALLCLK_CAMERA_LOG_PATH)
            self.writeListToCsv(self.PA_camera_frame_arrival_dt, "PA_camera_frame_publish_timestamp", PA_CAMERA_LOG_PATH)

    
            # end visual interface 
            self.PAC.disconnect()
            ctypes.windll.winmm.timeEndPeriod(1)
            os._exit(0)




if __name__ == "__main__":
    interceptor_interface = VisualizerKernel()
    interceptor_interface.manager()