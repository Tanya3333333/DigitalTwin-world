import asyncio, math
class PATargetDroneTrajectory:
    """Uses the Project AirSim API controls to do maneuvers for the target drones. 
    If you are using this class, use the jsonc files of drones with fastPhys extention in their names"""
    def __init__(self):
        self.ground_z = None
        self.ground_y = None
        self.ground_x = None
        self.ground_define = False
        self.going_up = True
        self.phase = None
        self.start_n = None
        self.hold_t = 0.0
        self.face_away_yaw = None


    async def verticle_occilation(self, drone, dt_s, climb_up_to=-3, verticle_speed=2):
        z = drone.get_ground_truth_pose()["translation"]["z"]

        if self.ground_z is None:
            self.ground_z = z                          # gets the initial height (once)

        top = self.ground_z + climb_up_to              # climb_up_to is negative -> above start

        # NED: up is negative z
        if z <= top:                 self.going_up = False   # hit the top -> descend
        elif z >= self.ground_z:     self.going_up = True    # back at start -> climb

        vel = -verticle_speed if self.going_up else verticle_speed
        await drone.move_by_velocity_async(
            v_north=0.0, v_east=0.0, v_down=vel, duration=dt_s
        )
 


    
 
    
    async def rectangle_mission(self, drone, climb_height=3.0, side=200.0, speed=3.0, pos_tol=0.5):
        """Climb, fly a square (N -> E -> S -> W), then return home and land.
 
        One position command per waypoint: the flight controller flies to each corner and
        DECELERATES into it, so corners stay crisp and "home" is actually reached. We issue
        the command once per waypoint (cmd_sent latch) and just watch ground-truth distance
        each step to know when to advance. Still steps in lockstep with the rest of the sim.
        """
        p = drone.get_ground_truth_pose()["translation"]
        x, y, z = p["x"], p["y"], p["z"]
 
        # build the waypoint list once, relative to the captured start (home) pose
        if not self.ground_define:
            x0, y0, z0 = x, y, z
            up = z0 - climb_height          # NED: up is more negative
            self.waypoints = [
                (x0,        y0,        up),   # 1. climb straight up
                (x0 + side, y0,        up),   # 2. +north
                (x0 + side, y0 + side, up),   # 3. +east
                (x0,        y0 + side, up),   # 4. back south (to home x)
                (x0,        y0,        up),   # 5. back west -> above home
                (x0,        y0,        z0),   # 6. descend -> land at home
            ]
            self.wp_i = 0
            self.cmd_sent = False
            self.ground_define = True
 
        # mission finished -> controller holds the last (home/ground) position
        if self.wp_i >= len(self.waypoints):
            return
 
        tx, ty, tz = self.waypoints[self.wp_i]
 
        # send the position command ONCE per waypoint; controller flies + settles there
        if not self.cmd_sent:
            await drone.move_to_position_async(tx, ty, tz, speed)
            self.cmd_sent = True
 
        # arrived at this corner? advance to the next one and re-arm the command
        dist = math.sqrt((tx - x) ** 2 + (ty - y) ** 2 + (tz - z) ** 2)
        if dist <= pos_tol:
            self.wp_i += 1
            self.cmd_sent = False
 
 
 

    
 
    async def vtol_and_cruise(self, drone, dt_s, climb_up_to=-3, cruise_dist=20, v_speed=6, cruise_speed =30, face_yaw_deg=180, hold_time=3.0):
        p = drone.get_ground_truth_pose()["translation"]
        if self.ground_z is None:
            self.ground_z = p["z"]
            self.start_n = p["x"]
            self.phase = "takeoff"
 
        top = self.ground_z + climb_up_to
 
        if self.phase == "takeoff":
            if p["z"] <= top:
                self.phase = "cruise"
            else: await drone.move_by_velocity_async(v_north=0.0, v_east=0.0, v_down=-v_speed, duration=dt_s)
 
 
        elif self.phase == "cruise":
            if p["x"] >= self.start_n + cruise_dist:
                self.phase = "rotate"
            else: await drone.move_by_velocity_async(v_north=cruise_speed, v_east=0.0, v_down=0.0, duration=dt_s)
 
 
        elif self.phase == "rotate":
            await drone.move_by_velocity_async(v_north=0.0, v_east=0.0, v_down=0.0, duration=dt_s,
                                               yaw_is_rate=False, yaw=math.radians(face_yaw_deg))
            self.phase = "cruise_back"
 
        elif self.phase == "cruise_back":
            # NOTE: still the unfixed version - v_north should be NEGATIVE and the test a
            # threshold (p["x"] <= start_n + tol), not == . Left as-is per your request.
            if p["x"] == self.start_n:
                self.phase = "land"
            await drone.move_by_velocity_async(v_north=cruise_speed, v_east=0.0, v_down=0.0, duration=dt_s)
 
 
        elif self.phase == "land":
            if p["z"] >= self.ground_z - 0.3:
                self.phase = "hold"
                self.hold_t = 0.0           # reset the timer when we land
            await drone.move_by_velocity_async(v_north=0.0, v_east=0.0, v_down=v_speed/2, duration=dt_s)
 
 
        elif self.phase == "hold":
            self.hold_t += dt_s             # count sim time
            if self.hold_t >= hold_time:    # elapsed -> start over
                self.phase = "takeoff"
                self.start_n = p["x"]       # re-anchor north for the next cruise
            await drone.move_by_velocity_async(v_north=0.0, v_east=0.0, v_down=0.0, duration=dt_s)  # sit still