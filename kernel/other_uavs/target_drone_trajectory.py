import math

class TargetDroneTrajectory:
    """This class is about different mission plan (trajectory models)
    (currently the dynamics are not driven by forces and moments)"""
    def __init__(self):

        self.t =  0 # s
        self.omega = 0.02 #rad/s
        
        self.yaw_angle = 0 #degree

        self.x = 60
        self.y = 0
        self.z = -5                                  
        self.qw = 1
        self.qx = 0
        self.qy = 0
        self.qz = 0

    def lawnmower_trajectory(self):
        """ a growing lawnmower motion (flight pattern for drones)"""
        dt = 0.01
        self.t += dt

        # constant forward motion
        v_x = 2 #0.04
        self.x = 60 + (v_x * self.t)
       

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
    

    def vertical_occilation_trajectory(self):
        dt = 0.01
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
    


    def depth_occilation_trajectory(self):
        dt = 0.01
        self.t += dt

        omega = 0.3
        self.x = (-10 + 125 * math.sin(omega * self.t))
        

        pose = { "frame_id": "DEFAULT_FRAME", 
                "translation": { "x": self.x, "y": self.y, "z": self.z }, 
                "rotation": { "w": self.qw, "x": self.qx, "y": self.qy, "z": self.qz } 
                }

        return pose
    

    def yz_circle_trajectory(self):


        dt = 0.01
        self.t += dt

        r = 3.0
        omega = 0.5

        x_center = 60
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

        pose = { "frame_id": "WORLD", 
                "translation": { "x": self.x, "y": self.y, "z": self.z }, 
                "rotation": { "w": self.qw, "x": self.qx, "y": self.qy, "z": self.qz } 
                }
        
        return pose
    

if __name__ == "__main__":
    target_motion = TargetDroneTrajectory()
    while True: 
        tar_pose = target_motion.yz_circle_trajectory()
        print (tar_pose)