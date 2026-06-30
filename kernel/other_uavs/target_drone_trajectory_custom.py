import math

class TargetDroneTrajectory:
    """This class is about different mission plan (trajectory models)
    (currently the dynamics are not driven by forces and moments)
    If you are using this class, use the jsonc files of drones without fastPhys extention in their names"""
    def __init__(self):

        self.t =  0 # s
        self.omega = 0.02 #rad/s
        
        self.yaw_angle = 0 #degree

        self.x = 20
        self.y = 5
        self.z = -4                              
        self.qw = 1
        self.qx = 0
        self.qy = 0
        self.qz = 0

        self.x_initial = 20
        self.y_inital = 0

    def lawnmower_trajectory(self, dt):
        """ a growing lawnmower motion (flight pattern for drones)"""
        self.t += dt

        # constant forward motion
        v_x = 2 #0.04
        self.x = self.x_initial + (v_x * self.t)
       

        # smooth sinosodal motion
        omega = 0.5 #0.03
        v_y = 2*omega * math.cos(omega * self.t)
        self.y = (10*math.sin(omega * self.t))
        
        self.z = -11
        # for the quaternion, assume the roll and pitch both zeros 
        yaw_angle = math.atan2(v_y, v_x)
        # (focus on yaw only - drone face the dirrection of movmenet)
        self.qw = math.cos(yaw_angle /2)
        self.qz = math.sin(yaw_angle /2)


        pose = { "frame_id": "DEFAULT_FRAME", 
                "translation": { "x": self.x, "y": self.y, "z": self.z }, 
                "rotation": { "w": self.qw, "x": self.qx, "y": self.qy, "z": self.qz } 
                }
        
        return pose
    

    def vertical_occilation_trajectory(self, dt):
        self.t += dt

        omega = 0.5
        self.z = (-3 + 5 * math.sin(omega * self.t))
        if self.z > -2: 
            self.z = -2

        pose = { "frame_id": "DEFAULT_FRAME", 
                "translation": { "x": self.x, "y": self.y, "z": self.z }, 
                "rotation": { "w": self.qw, "x": self.qx, "y": self.qy, "z": self.qz } 
                }

        return pose
    


    def depth_occilation_trajectory(self,dt):
        self.t += dt

        omega = 0.3
        self.x = (-10 + 125 * math.sin(omega * self.t))
        

        pose = { "frame_id": "DEFAULT_FRAME", 
                "translation": { "x": self.x, "y": self.y, "z": self.z }, 
                "rotation": { "w": self.qw, "x": self.qx, "y": self.qy, "z": self.qz } 
                }

        return pose
    

    def yz_circle_trajectory(self,dt):

        self.t += dt

        r = 3.0
        omega = 0.5

        x_center = 20
        y_center = 0
        z_center = -10   # NED: negative means above ground

        self.x = x_center
        self.y = y_center + r * math.cos(omega * self.t)
        self.z = z_center + r * math.sin(omega * self.t)

        pose = { "frame_id": "DEFAULT_FRAME", 
                "translation": { "x": self.x, "y": self.y, "z": self.z }, 
                "rotation": { "w": self.qw, "x": self.qx, "y": self.qy, "z": self.qz } 
                }

        return pose


    def hold_position(self):
        pose = { "frame_id": "DEFAULT_FRAME", 
                "translation": { "x": self.x, "y": self.y, "z": self.z }, 
                "rotation": { "w": self.qw, "x": self.qx, "y": self.qy, "z": self.qz } 
                }
        
        return pose
    

    def quintic(self, p0, pf, tau):
        """quintic solver for any initial and final position of trajectory that is time dependant(normalized)"""
        s = (
            10 * tau**3
            - 15 * tau**4
            + 6 * tau**5
        )
        return p0 + (pf - p0) * s
    
    def euler_to_quaternion(self, roll, pitch, yaw):

        cr = math.cos(roll / 2)
        sr = math.sin(roll / 2)

        cp = math.cos(pitch / 2)
        sp = math.sin(pitch / 2)

        cy = math.cos(yaw / 2)
        sy = math.sin(yaw / 2)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        return qw, qx, qy, qz
    
    def takeOff_landing_trajectory(self, dt, T_total):
        """full mission plan, 
        input: 
            T_total: set the total time of the mission
            dt: already set by the projectairsim stepping
        """
        self.t += dt
        tau_total = self.t % T_total

        # PHASE 1 : smooth climb and pitch up
        if tau_total < 8.0:
            tau = tau_total / 8.0

            x_final = self.x_initial + 20
            self.x = self.quintic(self.x_initial, (x_final + 20), tau)
            self.y = 0.0
            self.z = self.quintic(-5, -15, tau)
            roll = 0.0
            pitch = math.radians(15)
            yaw = 0.0

        # PHASE 2 : straight horizontal
        elif tau_total < 18.0:
            tau = (tau_total - 8.0) / 10.0

            x_inital = self.x_initial + 20
            x_final = x_inital + 20   
            self.x = self.quintic(x_inital, x_final, tau)
            self.y = 0.0
            self.z = -15.0
            roll = 0.0
            pitch = 0.0
            yaw = 0.0

        # PHASE 3 : smooth turning motion
        elif tau_total < 30.0:
            tau = (tau_total - 18.0) / 12.0
            theta = self.quintic(0.0, math.pi, tau)

            R = 20.0
            xc = self.x_initial + 40.0
            yc = 20.0

            self.x = xc + R * math.sin(theta)
            self.y = yc - R * math.cos(theta)
            self.z = -15.0
            roll = self.quintic(0.0, math.radians(20), tau)
            pitch = 0.0
            yaw = theta

        # PHASE 4 : return and land
        elif tau_total <= 40.0:
            tau = (tau_total - 30.0) / 10.0
            x_initial = self.x_initial + 40
            x_final = self.x_initial
            self.x = self.quintic(x_initial, x_final, tau)
            self.y = self.quintic(40, 0, tau)
            self.z = self.quintic(-15, -5, tau)
            roll = 0.0
            pitch = math.radians(-10)
            yaw = math.pi

        self.qw, self.qx, self.qy, self.qz = (self.euler_to_quaternion(roll,pitch,yaw))

        pose = { "frame_id": "DEFAULT_FRAME", 
                "translation": { "x": self.x, "y": self.y, "z": self.z }, 
                "rotation": { "w": self.qw, "x": self.qx, "y": self.qy, "z": self.qz } 
                }
        
        return pose
            