import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix, Imu

import math

def get_quaternion_from_euler(roll, pitch, yaw):
  """
  Convert an Euler angle to a quaternion.
   
  Imathut
    :param roll: The roll (rotation around x-axis) angle in radians.
    :param pitch: The pitch (rotation around y-axis) angle in radians.
    :param yaw: The yaw (rotation around z-axis) angle in radians.
 
  Output
    :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
  """
  qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
  qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
  qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
  qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
 
  return (qx, qy, qz, qw)

class Simulator(Node):

    def __init__(self):
        super().__init__('simulator')
        self.map_gps_publisher = self.create_publisher(NavSatFix, '/imu/nav_sat_fix', 10)
        self.goal_gps_subscriber = self.create_subscription(String, '/goal_gps_string', self.goal_gps_callback, 10)
        self.goal_gps_publisher = self.create_publisher(NavSatFix, '/goal_gps', 10)
        self.map_imu_publisher = self.create_publisher(Imu, '/imu/data', 10)
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
    
    def goal_gps_callback(self, msg):
        a = msg.data.split()
        msg2 = NavSatFix()
        msg2.latitude = float(a[0])
        msg2.longitude = float(a[1])
        self.goal_gps_publisher.publish(msg2)


    def timer_callback(self):
        msg = NavSatFix()
        msg.status.status = 0
        msg.latitude = 12.99
        msg.longitude = 80.23
        imu_msg = Imu()
        yaw = -0.0211145
        (imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z, imu_msg.orientation.w) = get_quaternion_from_euler(0, 0, yaw)
        self.map_gps_publisher.publish(msg)
        self.map_imu_publisher.publish(imu_msg)
        
        
def main(args=None):
    rclpy.init(args=args)

    simulator = Simulator()

    rclpy.spin(simulator)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    simulator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()