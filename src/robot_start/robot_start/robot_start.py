import rclpy
import math

from rclpy.node import Node
from rclpy.time import Time
from robot_start.robot_serial import RobotSerial
import tf_transformations   

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Float32


class RobotStart(Node):

    def __init__(self,name):

        super().__init__(name)

        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.vx = 0.0
        self.vth =0.0
        self.last_time = 0.0
        self.current_time =0.0
        self.dt =0.0

        self.power_valtage = 0.0
        self.offsetcount = 0
        self.Gyroscope_Xdata_Offset = 0.0
        self.Gyroscope_Ydata_Offset = 0.0
        self.Gyroscope_Zdata_Offset = 0.0


        self.filter_vx_match = 1.0
        self.filter_vth_match = 1.0
        self.OFFSET_COUNT = 40
        
        self.declare_parameter('usart_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_data', 115200)
        self.declare_parameter('robot_frame_id', 'base_link')
        # self.declare_parameter('smoother_cmd_vel', '/smoother_cmd_vel')
        self.declare_parameter('filter_vx_match',1.0)
        self.declare_parameter('filter_vth_match', 1.0)

        self.usart_port = self.get_parameter('usart_port').value
        self.baud_data = self.get_parameter('baud_data').value
        self.robot_frame_id = self.get_parameter('robot_frame_id').value
        self.filter_vx_match = self.get_parameter('filter_vx_match').value
        self.filter_vth_match = self.get_parameter('filter_vth_match').value


        self.robot_serial= RobotSerial(self.usart_port,self.baud_data)
        self.mpu6050 = Imu()

        self.power_pub = self.create_publisher(Float32,'/robot/powervaltage',20)
        self.odom_pub = self.create_publisher(Odometry,'odom',50)
        self.imu_pub = self.create_publisher(Imu,'/mobile_base/sensors/imu_data',20)
        self.imu_raw_pub = self.create_publisher(Imu,'/mobile_base/sensors/imu_data_raw',20)
        self.odom_pose_covariance =    [1e-3, 0, 0, 0, 0, 0, 
                                        0, 1e-3, 0, 0, 0, 0,
                                        0, 0, 1e6, 0, 0, 0,
                                        0, 0, 0, 1e6, 0, 0,
                                        0, 0, 0, 0, 1e6, 0,
                                        0, 0, 0, 0, 0, 1e3]
        self.odom_pose_covariance2 =   [1e-9, 0, 0, 0, 0, 0, 
									    0, 1e-3, 1e-9, 0, 0, 0,
										0, 0, 1e6, 0, 0, 0,
										0, 0, 0, 1e6, 0, 0,
										0, 0, 0, 0, 1e6, 0,
										0, 0, 0, 0, 0, 1e-9]
        self.odom_twist_covariance =   [1e-3, 0, 0, 0, 0, 0, 
                                        0, 1e-3, 0, 0, 0, 0,
                                        0, 0, 1e6, 0, 0, 0,
                                        0, 0, 0, 1e6, 0, 0,
                                        0, 0, 0, 0, 1e6, 0,
                                        0, 0, 0, 0, 0, 1e3]
        self.odom_twist_covariance2 =  [1e-9, 0, 0, 0, 0, 0, 
                                        0, 1e-3, 1e-9, 0, 0, 0,
                                        0, 0, 1e6, 0, 0, 0,
                                        0, 0, 0, 1e6, 0, 0,
                                        0, 0, 0, 0, 1e6, 0,
                                        0, 0, 0, 0, 0, 1e-9]
        

# 如果需要使用该数组，只需使用 odom_pose_covariance 即可

     # self.subscription = self.create_subscription(
        #     ProtocolUploadData,
        #     'upload_data_topic',  # 替换成实际的消息话题名称
        #     self.data_callback,
        #     10  # 可以根据需要调整队列大小
        # )
        # 创建发布者对象（消息类型、话题名、队列长度）
        # self.pub = self.create_publisher(String, "chatter", 10)

    def robotstart_loopprocess(self):
        self.last_time  =Time.now()
        while rclpy.ok():
            self.current_time =Time.now()
            self.dt = (self.current_time-self.last_time).toSec()

            if (self.robot_serial.protocol_data_receive() == True):
                self.serial_data_assignment()

                delta_x = self.vx * math.cos(self.th ) * self.dt
                delta_y = self.vx * math.sin(self.th ) * self.dt
                delta_th = self.vth * self.dt
                self.x += delta_x
                self.y += delta_y
                self.th += delta_th
                
                if (self.offsetcount < self.OFFSET_COUNT):
                    self.offsetcount +=1
                    # self.accelerometeroffset(self.mpu6050.angular_velocity.x,self.mpu6050.angular_velocity.y,self.mpu6050.angular_velocity.z)                
                else:
                    self.offsetcount = self.OFFSET_COUNT
                    self.mpu6050.angular_velocity.x -= self.Gyroscope_Xdata_Offset
                    self.mpu6050.angular_velocity.y -= self.Gyroscope_Ydata_Offset
                    self.mpu6050.angular_velocity.z -= self.Gyroscope_Zdata_Offset
                    
                    self.publisherpower()
                    self.publisherodom()
                    self.publisherimusensor()
                    self.publisherimusensorraw()
            
            self.last_time = self.current_time
            rclpy.spin_once()
   
    def serial_data_assignment(self):

        self.vx =self.robot_serial.receive_str.sensor_str.x_speed *self.filter_vx_match
        self.vth = self.robot_serial.receive_str.sensor_str.z_speed *self.filter_vth_match
        self.power_valtage = self.robot_serial.receive_str.sensor_str.source_voltage 

        self.mpu6050.linear_acceleration.x = self.robot_serial.receive_str.link_accelerometer.x_data
        self.mpu6050.linear_acceleration.y = self.robot_serial.receive_str.link_accelerometer.y_data
        self.mpu6050.linear_acceleration.z = self.robot_serial.receive_str.link_accelerometer.z_data
        
        self.mpu6050.angular_velocity.x = self.robot_serial.receive_str.link_gyroscope.x_data
        self.mpu6050.angular_velocity.y = self.robot_serial.receive_str.link_gyroscope.y_data
        self.mpu6050.angular_velocity.z = self.robot_serial.receive_str.link_gyroscope.z_data

    def publisherpower(self):
        
        power_msgs  = Float32()
        power_msgs.data = self.power_valtage
        self.power_pub.publish(power_msgs)

    def publisherodom(self):

        odom_quat = Quaternion()
        odom_quat = tf_transformations.quaternion_from_euler(0, 0, self.th)
        odom = Odometry()
        odom.header.stamp = Time.now()
        odom.header.frame_id = "odom"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = odom_quat

        odom.child_frame_id = self.robot_frame_id
        odom.twist.twist.linear.x =  self.vx
        odom.twist.twist.linear.y =  0.0
        odom.twist.twist.angular.z = self.vth

        if(self.vx == 0):
            odom.pose.covariance = self.odom_pose_covariance2
            odom.twist.covariance = self.odom_twist_covariance2
        else:
            odom.pose.covariance = self.odom_pose_covariance
            odom.twist.covariance = self.odom_twist_covariance
        
        self.odom_pub.publish(odom)

    def publisherimusensor(self):
        pass
    def publisherimusensorraw(self):
        pass


def main(args=None):
    rclpy.init(args=args)
    node = RobotStart("robot_start")
    node.robotstart_loopprocess()
    rclpy.shutdown()