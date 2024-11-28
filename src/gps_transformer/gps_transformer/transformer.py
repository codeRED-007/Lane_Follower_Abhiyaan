import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import PoseStamped


import math

def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians

class GoalCoordinatePublisher(Node):

    def __init__(self):
        super().__init__('goal_pose_publisher')
        self.bot_gps_reciever = self.create_subscription(NavSatFix, '/gps/fix', self.bot_gps_callback, 10)
        self.goal_gps_receiver = self.create_subscription(NavSatFix, '/goal_gps_start', self.goal_gps_callback1, 10)
        self.goal_gps_receiver = self.create_subscription(NavSatFix, '/goal_gps_end', self.goal_gps_callback2, 10)
        self.imu_reciever = self.create_subscription(Imu, '/imu_plugin/out', self.imu_callback,10)
        
        self.goal_publisher1 = self.create_publisher(PoseStamped, '/goal_pose_start', 10)
        self.goal_publisher2 = self.create_publisher(PoseStamped, '/goal_pose_end', 10)

        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.map_heading = 0
        self.map_heading_set = False
        self.goal_gps1 = NavSatFix()
        self.goal_gps2 = NavSatFix() # gps coordinates of the goal point
        self.map_gps = NavSatFix() # gps coordinates of the origin of the map frame
        self.goal_msg1 = PoseStamped()
        self.goal_msg2 = PoseStamped()
        
        self.map_angle = 0 # Angle that the map frame y-axis makes with geographic North
        self.map_angle_set = False
        self.map_gps_set = False
        self.geonorth_vector1 = (0, 0)
        self.magnorth_vector1 = (0, 0)
        self.mapframe_vector1 = (0, 0)
        self.geonorth_vector2 = (0, 0)
        self.magnorth_vector2 = (0, 0)
        self.mapframe_vector2 = (0, 0)
        self.earth_radius = 63781370
        self.magnetic_declination = 0
    
    def timer_callback(self):
        self.goal_msg1.pose.position.x = float(self.mapframe_vector1[0])
        self.goal_msg1.pose.position.y = float(self.mapframe_vector1[1])
        self.goal_msg1.header.stamp = self.get_clock().now().to_msg()
        self.goal_msg2.pose.position.x = float(self.mapframe_vector2[0])
        self.goal_msg2.pose.position.y = float(self.mapframe_vector2[1])
        self.goal_msg2.header.stamp = self.get_clock().now().to_msg()
        self.goal_publisher1.publish(self.goal_msg1)
        self.goal_publisher2.publish(self.goal_msg2)
        # print(f"Geonorth vector : {self.geonorth_vector[0]}, {self.geonorth_vector[1]}")

    def bot_gps_callback(self, msg):
        if(self.map_gps_set == True):
            return
        if(msg.status.status == 0):
            self.map_gps_set = True
            self.map_gps.latitude = msg.latitude
            self.map_gps.longitude = msg.longitude
            print("Map GPS point set")
            self.get_radius()
            return
    
    def goal_gps_callback1(self, msg):
        self.goal_gps1.latitude = msg.latitude
        self.goal_gps1.longitude = msg.longitude
        self.get_geonorth_vector1()
        self.get_magnorth_vector1()
        self.get_map_vector1()
        return
    def goal_gps_callback2(self, msg):
        self.goal_gps2.latitude = msg.latitude
        self.goal_gps2.longitude = msg.longitude
        self.get_geonorth_vector2()
        self.get_magnorth_vector2()
        self.get_map_vector2()
        return

    def get_radius(self):
        self.earth_radius = 6371000
        return

    def imu_callback(self, msg):
        # code to get the magnetometer readings from imu data and use it to calculate the angle that
        # the map y axis makes with geographic north

        if self.map_heading_set==True:
            return
        (x, y, z, w) = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        self.map_heading = euler_from_quaternion(x, y, z, w)[2]
        self.map_heading_set = True
        print("Map heading set")

        return
    

    def get_geonorth_vector1(self):
        # gets the x, y coordinates of the goal points with respect to a frame with origin at
        # map frame origin but with y-axis along geographic north

        x = self.earth_radius * math.cos(math.radians(self.goal_gps1.latitude) )* (math.radians(self.goal_gps1.longitude - self.map_gps.longitude))
        y = self.earth_radius * (math.radians(self.goal_gps1.latitude  - self.map_gps.latitude))
        self.geonorth_vector1 = (x, y)
        return
    
    def get_geonorth_vector2(self):
        # gets the x, y coordinates of the goal points with respect to a frame with origin at
        # map frame origin but with y-axis along geographic north

        x = self.earth_radius * math.cos(math.radians(self.goal_gps2.latitude) )* (math.radians(self.goal_gps2.longitude - self.map_gps.longitude))
        y = self.earth_radius * (math.radians(self.goal_gps2.latitude  - self.map_gps.latitude))
        self.geonorth_vector2 = (x, y)
        return

    def get_magnorth_vector1(self):
        self.magnorth_vector1 = self.rotate(self.geonorth_vector1, self.magnetic_declination)

    def get_magnorth_vector2(self):
        self.magnorth_vector2 = self.rotate(self.geonorth_vector2, self.magnetic_declination)
    
    def get_map_vector1(self):
        self.mapframe_vector1 = self.rotate(self.magnorth_vector1, -self.map_heading)

    def get_map_vector2(self):
        self.mapframe_vector2 = self.rotate(self.magnorth_vector2, -self.map_heading)


    def rotate(self, vector, angle):
        # angle is taken in radians
        c = math.cos(angle)
        s = math.sin(angle)
        new_vector = (vector[0]*c - vector[1]*s, vector[0]*s + vector[1]*c)
        return new_vector
    



def main(args=None):
    rclpy.init(args=args)

    goal_pose_publisher = GoalCoordinatePublisher()

    rclpy.spin(goal_pose_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    goal_pose_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '_main_':
    main()