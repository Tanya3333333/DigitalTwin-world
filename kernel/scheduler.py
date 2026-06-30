import os
os.environ["FOR_DISABLE_CONSOLE_CTRL_HANDLER"] = "1"

import torch
print(f"[torch-cuda is available]: {torch.cuda.is_available()}")

import os, csv, time, ctypes, asyncio, signal
import multiprocessing as mp

from projectairsim import ProjectAirSimClient, World, Drone

from kernel.interceptor.main_uav_state_server import interceptor_udp_receiver
from kernel.other_uavs.target_drone_trajectory_custom import TargetDroneTrajectory
from kernel.other_uavs.target_drone_trajectory_PA_api import PATargetDroneTrajectory
from ground_station.target_status_client import SendTargetStatus

from companion_computer.detection_and_track.camera_track.yolo_track import YOLOTrack
from kernel.interceptor.camera_publish_callback import opencv_mp, opencv_vid_mp


# log paths
desktop = os.path.expanduser("~/Desktop")

RT_RATIO_LOG_PATH = os.path.join(desktop, "rt_ratio.csv")
FULL_LOOP_DT_LOG_PATH = os.path.join(desktop, "full_loop_dt.csv")
PA_FULL_LOOP_DT_LOG_PATH = os.path.join(desktop, "pa_full_loop_dt.csv")
PA_INTERNAL_STEP_DT_LOG_PATH = os.path.join(desktop, "pa_internal_step_dt.csv")
STEP_DRONES_STATE_DT_LOG_PATH = os.path.join(desktop, "step_drones_states_dt.csv")
CAMERA_PUBLISH_WALLCLK_DT_LOG_PATH = os.path.join(desktop, "camera_publish_wallclk_dt.csv")
CAMERA_PUBLISH_PA_INTERNAL_DT_LOG_PATH = os.path.join(desktop, "camera_publish_pa_internal_dt.csv")

INTERCEPTOR_STATE_CSV = os.path.join(desktop, "interceptor_state.csv")
TARGET1_STATE_CSV = os.path.join(desktop, "targetdrone1_state.csv")
CAMERA_STATE_CSV = os.path.join(desktop, "camera_state.csv")

# Constants for testing purposes (otherwise the dt step of sim can be read from the sim_config automatically)
EXPECTED_DT = 0.01

# signals
def _camera_display_worker(frame_queue):
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    opencv_vid_mp.show_frame_opencv(frame_queue)
 
def _yolo_track_worker(track_queue):
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    YOLOTrack.show_tracked_frame_opencv(track_queue)
 
def _udp_worker(shared_state, stop_event):
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    interceptor_udp_receiver(shared_state, stop_event)


 


class VisualizerKernel:
    def __init__(self):

        # initialize connection with Unreal / Project AirSim
        self.PAC = ProjectAirSimClient()
        self.PAC.connect()


        # initialize world and drones:
        self.world = World(self.PAC, "scene_for_drones_with_fastPhys_models.jsonc", delay_after_load_sec=2)
        self.interceptor = Drone(self.PAC, self.world, "interceptor")
        self.targetDrone1 = Drone(self.PAC, self.world, "target1")
        self.targetDrone2 = Drone(self.PAC, self.world, "target2")
        self.targetDrone3 = Drone(self.PAC, self.world, "target3")
        self.expected_dt = self.world.sim_config["clock"]["step-ns"] # get step-ns from config

                # cooperative-stop flag + guards so shutdown only runs once
        self._stop = False
        self._shutdown_done = False
 
 
        #processes and threads: shared interceptor state, udp, camera sensor variables, YOLO_tracker
        self.manager_mp = mp.Manager()
        self.shared_state_interceptor = self.manager_mp.dict()
        self.udp_stop_event = mp.Event()
        self.udp_process = mp.Process(
            target=_udp_worker,
            args=(self.shared_state_interceptor, self.udp_stop_event),
        )
        # the camera-display process and the YOLO-track process each need their own queue; otherwise, they stole frames from each other
        self.frame_queue = mp.Queue(maxsize=5)   # -> camera display
        self.track_queue = mp.Queue(maxsize=5)   # -> YOLO tracker
 
        self.camera_frame_process = mp.Process(
            target=_camera_display_worker,
            args=(self.frame_queue,),
        )
        self.track_process = mp.Process(
            target=_yolo_track_worker,
            args=(self.track_queue,),
        )


 






        # drones maneuvers: interceptor, target drones [1,2,3, ...]
        self.shared_state_interceptor["pose"] = {
            "frame_id": "DEFAULT_FRAME",
            "translation": {"x": 0, "y": 0, "z": 0},
            "rotation": {"w": 1, "x": 0, "y": 0, "z": 0},
        }
        
        self.target_uav1_maneuver = PATargetDroneTrajectory()
        ok1 = self.targetDrone1.enable_api_control()
        ok2 = self.targetDrone1.arm()
        print("[target1] api_control:", ok1, " arm:", ok2)
        self.targetDrone1_stat = SendTargetStatus()


        self.target_uav2_maneuver = PATargetDroneTrajectory()
        ok3 = self.targetDrone2.enable_api_control()
        ok4 = self.targetDrone2.arm()
        print("[target2] api_control:", ok3, " arm:", ok4)

        
        self.target_uav3_maneuver = TargetDroneTrajectory()
        self.latest_pose_target_uav3 = {
            "frame_id": "DEFAULT_FRAME",
            "translation": {"x": 20, "y": -5, "z": -4},
            "rotation": {"w": 1, "x": 0, "y": 0, "z": 0},
        }
        
        self.latest_target_poses = [
            self.latest_pose_target_uav3,
        ]



        # timing analysis lists/variables:
        self.rt_ratio = []
        self.full_loop_dt = []
        self.pa_full_loop_dt = []
        self.pa_internal_step_dt = []
        self.step_drones_states_dt = []
        self.camera_publish_wallclk_dt = []
        self.camera_publish_pa_internal_dt = []
        self.last_wall_ts = None
        self.last_sim_ts = None
        try: # increase time resolution for more determinsitic reaction to time related variables
            ctypes.windll.winmm.timeBeginPeriod(1) 
            print("[Timer] Windows timer resolution set to 1 ms")
        except Exception as e: print("[Timer] Could not set Windows timer resolution:", e)


        # variables for training YOLO model with range as one of the features
        self.interceptor_state_log = []
        self.targetDrone1_state_log = []
        self.camera_state_log = []

        
### callbacks



    @staticmethod
    def _push_drop_oldest(q, item):
        """Keep only the most recent frames; never block the publisher."""
        if q.full():
            try: q.get_nowait()
            except Exception: pass
        try: q.put_nowait(item)
        except Exception: pass
 
    def camera_frame_callback(self, topic, msg):
        
        if msg is None: return
        self.camera_state_log.append(self.extract_camera_state_row(msg))
        
        # timing analysis
        now_wall = time.perf_counter_ns()
        sim_ts = msg["time_stamp"]
        if self.last_wall_ts is not None and self.last_sim_ts is not None:
            camera_wall_dt = (now_wall - self.last_wall_ts)
            camera_internal_pa_dt = (sim_ts - self.last_sim_ts)
            self.camera_publish_wallclk_dt.append(camera_wall_dt)
            self.camera_publish_pa_internal_dt.append(camera_internal_pa_dt)
        self.last_wall_ts = now_wall
        self.last_sim_ts = sim_ts
 
        # fan the frame out to BOTH consumers, each on its own queue,
        # keeping only recent frames so neither queue grows forever
        self._push_drop_oldest(self.frame_queue, msg)
        self._push_drop_oldest(self.track_queue, msg)
        

    # def camera_frame_callback(self, topic, msg):
        
    #     if msg is None: return
    #     self.camera_state_log.append(self.extract_camera_state_row(msg))
        
    #     # timing analysis
    #     now_wall = time.perf_counter_ns()
    #     sim_ts = msg["time_stamp"]
    #     if self.last_wall_ts is not None and self.last_sim_ts is not None:
    #         camera_wall_dt = (now_wall - self.last_wall_ts)
    #         camera_internal_pa_dt = (sim_ts - self.last_sim_ts)
    #         self.camera_publish_wallclk_dt.append(camera_wall_dt)
    #         self.camera_publish_pa_internal_dt.append(camera_internal_pa_dt)
    #     self.last_wall_ts = now_wall
    #     self.last_sim_ts = sim_ts

    #     # keep recent frames and does not let queue grow forever
    #     if self.frame_queue.full():              
    #         try: self.frame_queue.get_nowait()  
    #         except: pass
    #     self.frame_queue.put(msg)
        

    def interceptor_state_callback(self, topic, msg):
        if msg is not None:
            self.interceptor_state_log.append(self.extract_pose_state_row(msg))

    def targetDrone1_state_callback(self, topic, msg):
        if msg is not None:
            self.targetDrone1_state_log.append(self.extract_pose_state_row(msg))
            posi = [msg["position"]["x"], msg["position"]["y"], msg["position"]["z"]]
            self.targetDrone1_stat.current_target_posi(posi)
    
    def targetDrone1_collision_callback(self, topic, msg):
        if msg is None: return
        #print("collision target 1: ", msg)
        self.targetDrone1_stat.collision_monitor_with_interceptor(1, msg["object_name"])
    
### action items

    def compute_next_target_uav_poses(self, dt_s):
        #pose2 = self.target_uav2_maneuver.takeOff_landing_trajectory(dt_s, 40)
        pose3 = self.target_uav3_maneuver.takeOff_landing_trajectory(dt_s, 40)
        
        #pose2["translation"]["y"] -= 5
        pose3["translation"]["y"] += 5
        return [pose3]

    def step_drones(self):
        self.interceptor.set_pose(self.shared_state_interceptor["pose"])
        #self.targetDrone2.set_pose(self.latest_target_poses[0])
        self.targetDrone3.set_pose(self.latest_target_poses[0])



### main function
    async def main(self):

        # start processes/threads: 
        self.udp_process.start()
        self.camera_topic = "/Sim/SceneBasicDrone/robots/interceptor/sensors/InterceptionCamera/scene_camera"
        self.PAC.subscribe(self.camera_topic, self.camera_frame_callback)
        self.camera_frame_process.start()
        self.PAC.subscribe(self.interceptor.robot_info["actual_pose"], self.interceptor_state_callback)
        self.PAC.subscribe(self.targetDrone1.robot_info["actual_pose"], self.targetDrone1_state_callback)
        self.PAC.subscribe(self.targetDrone1.robot_info["collision_info"], self.targetDrone1_collision_callback)
        self.track_process.start()
        
        try:
            dt_s = self.expected_dt * 1e-9
            while True:
                full_start = time.perf_counter_ns()
                pa_full_start = self.world.get_sim_time()

                step_drones_start = time.perf_counter_ns()
                await self.target_uav1_maneuver.rectangle_mission(self.targetDrone1)
                await self.target_uav2_maneuver.verticle_occilation(self.targetDrone2, dt_s)
                self.step_drones()
                step_drones_end = time.perf_counter_ns()
                
                
                pa_internal_step_start = time.perf_counter_ns()
                self.world.continue_for_single_step(wait_until_complete=True)
                pa_internal_step_end = time.perf_counter_ns()


                self.latest_target_poses = self.compute_next_target_uav_poses(dt_s)


                full_end = time.perf_counter_ns()
                pa_full_end = self.world.get_sim_time()


                # timing evaluations
                rt_ratio = self.expected_dt / (full_end - full_start)
                full_loop_dt = (full_end - full_start) 
                pa_full_loop_dt = (pa_full_end - pa_full_start)
                walkClk_pa_internal_step_dt = (pa_internal_step_end - pa_internal_step_start) 
                step_drones_state_dt = step_drones_end - step_drones_start
               
                self.rt_ratio.append(rt_ratio)
                self.full_loop_dt.append(full_loop_dt)
                self.pa_full_loop_dt.append(pa_full_loop_dt)
                self.pa_internal_step_dt.append(walkClk_pa_internal_step_dt)
                self.step_drones_states_dt.append(step_drones_state_dt)

        except (KeyboardInterrupt, asyncio.CancelledError, EOFError, BrokenPipeError):
            print("Stopping sim...")

    
        finally:

            if self._shutdown_done:return
            self._shutdown_done = True

            try: signal.signal(signal.SIGINT, signal.SIG_IGN)
            except Exception: pass

            for action in (
                self.targetDrone1.disarm,
                self.targetDrone2.disarm,
                self.targetDrone1.disable_api_control,
                self.targetDrone2.disable_api_control,
                self.targetDrone1_stat.end_thread,
            ):
                try: action()
                except Exception as e: print(f"[shutdown] {action.__name__} failed: {e}")

            # stop all processes/threads/clean queues 
            try: self.PAC.unsubscribe(self.camera_topic)
            except Exception as e: print(f"[shutdown] unsubscribe failed: {e}")
 
            try: self.frame_queue.put_nowait(None)
            except Exception: pass
            try: self.track_queue.put_nowait(None)
            except Exception: pass
 
            self._stop_process(self.camera_frame_process, "camera_frame_process")
            self._stop_process(self.track_process, "track_process")
            self.udp_stop_event.set()
            self._stop_process(self.udp_process, "udp_process")

            # disconnect to renderer (Unreal)
            try: self.PAC.disconnect()
            except Exception as e: print(f"[shutdown] disconnect failed: {e}")

            try: ctypes.windll.winmm.timeEndPeriod(1)
            except Exception: pass


            # disconnect to renderer (Unreal)
            try: self.PAC.disconnect()
            except Exception as e: print(f"[shutdown] disconnect failed: {e}")

            # mop up anything disconnect() itself may have scheduled, and let
            # pynng finalizers run while exceptions are still being suppressed
            try:
                pending = [t for t in asyncio.all_tasks()
                           if t is not asyncio.current_task()]
                for t in pending:
                    t.cancel()
                if pending:
                    await asyncio.gather(*pending, return_exceptions=True)
            except Exception:
                pass

            try: ctypes.windll.winmm.timeEndPeriod(1)
            except Exception: pass

          


            
            # write logs
            self.writeListToCsv(self.rt_ratio, "rt_ratio", RT_RATIO_LOG_PATH)
            self.writeListToCsv(self.full_loop_dt, "full_loop_dt", FULL_LOOP_DT_LOG_PATH)
            self.writeListToCsv(self.pa_full_loop_dt, "pa_full_loop_dt" , PA_FULL_LOOP_DT_LOG_PATH)
            self.writeListToCsv(self.pa_internal_step_dt, "pa_internal_step_dt", PA_INTERNAL_STEP_DT_LOG_PATH)
            self.writeListToCsv(self.step_drones_states_dt, "step_drones_states_dt", STEP_DRONES_STATE_DT_LOG_PATH)
            self.writeListToCsv(self.camera_publish_wallclk_dt, "camera_publish_wallclk_dt", CAMERA_PUBLISH_WALLCLK_DT_LOG_PATH)
            self.writeListToCsv(self.camera_publish_pa_internal_dt, "camera_publish_pa_internal_dt", CAMERA_PUBLISH_PA_INTERNAL_DT_LOG_PATH)


            # Detection team request for training YOLO with drones feature and ranges
            self.writeStateLogToCsv(self.interceptor_state_log, INTERCEPTOR_STATE_CSV)
            self.writeStateLogToCsv(self.targetDrone1_state_log, TARGET1_STATE_CSV)
            self.writeStateLogToCsv(self.camera_state_log, CAMERA_STATE_CSV)

            print("Shutdown complete.")


### helpers 

    @staticmethod
    def _stop_process(proc, name):
        """Escalating, exception-safe teardown: join -> terminate -> kill."""
        if proc is None:
            return
        try:
            if proc.is_alive():
                proc.join(timeout=3)
            if proc.is_alive():
                print(f"[shutdown] {name} still alive, terminating")
                proc.terminate()
                proc.join(timeout=2)
            if proc.is_alive():
                print(f"[shutdown] {name} still alive, killing")
                proc.kill()
                proc.join(timeout=2)
        except Exception as e:
            print(f"[shutdown] error stopping {name}: {e}")


    def extract_pose_state_row(self, msg):
        return {
            "projectairsim_time_stamp_ns": msg["time_stamp"],
            "unix_wall_time_s": time.time(),
            "position_x": msg["position"]["x"],
            "position_y": msg["position"]["y"],
            "position_z": msg["position"]["z"],
            "orientation_w": msg["orientation"]["w"],
            "orientation_x": msg["orientation"]["x"],
            "orientation_y": msg["orientation"]["y"],
            "orientation_z": msg["orientation"]["z"],
        }

    def extract_camera_state_row(self, msg):
        return {
            "projectairsim_time_stamp_ns": msg["time_stamp"],
            "unix_wall_time_s": time.time(),
            "height": msg["height"],
            "width": msg["width"],
            "encoding": msg["encoding"],
            "pos_x": msg["pos_x"],
            "pos_y": msg["pos_y"],
            "pos_z": msg["pos_z"],
            "rot_w": msg["rot_w"],
            "rot_x": msg["rot_x"],
            "rot_y": msg["rot_y"],
            "rot_z": msg["rot_z"],
        }


    def writeStateLogToCsv(self, rows, file_path):
        if not rows:
            print(f"No data to write for {file_path}")
            return

        header = list(rows[0].keys())

        with open(file_path, "w", newline="") as file:
            writer = csv.DictWriter(file, fieldnames=header)
            writer.writeheader()
            writer.writerows(rows)

        print(f"Successfully wrote state log to {file_path}")

    
    def writeListToCsv(self, dataList, header, fileDir):
        with open(fileDir, "w", newline="") as file:
            writer = csv.writer(file)
            writer.writerow([header])

            for val in dataList:
                writer.writerow([val])

        print(f"Successfully wrote data to {fileDir}") 




if __name__ == "__main__":
    interceptor_interface = VisualizerKernel()
    asyncio.run(interceptor_interface.main())