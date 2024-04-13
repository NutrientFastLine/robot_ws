import serial
import struct
from robot_interfaces.msg import ProtocolUploadData

class RobotSerial():

    def __init__(self,usart_port,baud_data):

        self.serial_com = serial.Serial()
        self.send_str = ProtocolUploadData()
        self.receive_str = ProtocolUploadData()
        self.ProtocolSize = 33

        self.serial_com.port = usart_port
        self.serial_com.baudrate = baud_data
        self.serial_com.timeout = 0.01
        self.serial_com.open()
    

    def protocol_data_assignment(self,msg,x_speed,z_speed):

        msg.sensor_str.header   = 0xFEFEFEFE
        msg.sensor_str.end_flag = 0xEE
        msg.sensor_str.x_speed  = x_speed
        msg.sensor_str.y_speed  = 0.0
        msg.sensor_str.z_speed  = z_speed
        msg.sensor_str.source_voltage = 0.0
        msg.sensor_str.link_accelerometer.x_data = 0
        msg.sensor_str.link_accelerometer.x_data = 0
        msg.sensor_str.link_accelerometer.x_data = 0
        msg.sensor_str.link_gyroscope.x_data = 0
        msg.sensor_str.link_gyroscope.x_data = 0
        msg.sensor_str.link_gyroscope.x_data = 0
        protocol_data_send = struct.pack("IffffhhhhhhB",
                            msg.sensor_str.header,
                            msg.sensor_str.x_speed,
                            msg.sensor_str.y_speed,
                            msg.sensor_str.z_speed,
                            msg.sensor_str.source_voltage,
                            msg.sensor_str.link_accelerometer.x_data,
                            msg.sensor_str.link_accelerometer.x_data,
                            msg.sensor_str.link_accelerometer.x_data,
                            msg.sensor_str.link_gyroscope.x_data,
                            msg.sensor_str.link_gyroscope.x_data,
                            msg.sensor_str.link_gyroscope.x_data,
                            msg.sensor_str.end_flag)

        return protocol_data_send

    def protocol_data_receive(self):
        # 在这里处理接收到的消息
        # self.count = self.serial_com .in_waiting
        # self.get_logger().info("data : %d" %self.count)
        counter = 0
        receive_flag = 0
        # self.get_logger().info("receive_flag: {}".format(receive_flag))
        while not receive_flag:
            if counter == 0:
                received_data_bytes = self.serial_com.read(1)
                if received_data_bytes == b'\xFE':
                    counter += 1
                    # self.get_logger().info("received_data_bytes first byte is 0xFE: {}".format(received_data_bytes))
                else:
                    counter = 0 
                    # self.get_logger().info("received_data_bytes first byte not 0xFE: {}".format(received_data_bytes))               
            elif counter == 1:
                received_data_bytes += self.serial_com.read(3)
                if received_data_bytes == b'\xFE\xFE\xFE\xFE':
                    counter += 1
                    # self.get_logger().info("received_data_bytes four byte is 0xFEFEFEFE: {}".format(received_data_bytes))
                else:
                    counter = 0
                    # self.get_logger().info("Header verification failed") 
                    # self.get_logger().info("received_data_bytes four byte not 0xFEFEFEFE: {}".format(received_data_bytes))
            elif counter == 2: 
                received_data_bytes += self.serial_com.read(29)
                if received_data_bytes[-1:] == b'\xEE':
                    receive_flag = 1
                    # self.get_logger().info("received_data_bytes last byte is 0xEE: {}".format(received_data_bytes[-1:]))
                    # self.get_logger().info("Protocol verification success!!")
                    # self.get_logger().info("receive_flag: {}".format(receive_flag))
                else:
                    counter = 0
                    # self.get_logger().info("Tail verification failed") 
                    # self.get_logger().info("received_data_bytes last byte not 0xEE: {}".format(received_data_bytes[-1:]))
            else:
                counter = 0
            
            if receive_flag == 1:
                # self.get_logger().info("Start unpack ProtocolUploadData message: {}".format(received_data_bytes))

                self.receive_str.buffer[:self.ProtocolSize] = received_data_bytes
                # unpack_data = struct.unpack('<IffffhhhhhhB',received_data_bytes)
                self.receive_str.sensor_str.header     = struct.unpack('I', received_data_bytes[0:4])[0]
                self.receive_str.sensor_str.end_flag = struct.unpack('B', received_data_bytes[32:33])[0]
                self.receive_str.sensor_str.x_speed    = struct.unpack('f', received_data_bytes[4:8])[0]
                self.receive_str.sensor_str.y_speed = struct.unpack('f', received_data_bytes[8:12])[0]
                self.receive_str.sensor_str.z_speed = struct.unpack('f', received_data_bytes[12:16])[0]
                self.receive_str.sensor_str.source_voltage = struct.unpack('f', received_data_bytes[16:20])[0]
                self.receive_str.sensor_str.link_accelerometer.x_data = struct.unpack('h', received_data_bytes[20:22])[0]
                self.receive_str.sensor_str.link_accelerometer.y_data = struct.unpack('h', received_data_bytes[22:24])[0]
                self.receive_str.sensor_str.link_accelerometer.z_data = struct.unpack('h', received_data_bytes[24:26])[0]
                self.receive_str.sensor_str.link_gyroscope.x_data = struct.unpack('h', received_data_bytes[26:28])[0]
                self.receive_str.sensor_str.link_gyroscope.y_data = struct.unpack('h', received_data_bytes[28:30])[0]
                self.receive_str.sensor_str.link_gyroscope.z_data = struct.unpack('h', received_data_bytes[30:32])[0]
                
        return True
        # self.get_logger().info("Verification success")

    # def protocol_data_print(self,msg):
    #     # 打印ProtocolUploadData消息中的所有变量值
    #     self.get_logger().info("Buffer V: {}".format(msg.buffer))
    #     self.get_logger().info("Header: {}".format(msg.sensor_str.header))
    #     self.get_logger().info("X Speed: {}".format(msg.sensor_str.x_speed))
    #     self.get_logger().info("Y Speed: {}".format(msg.sensor_str.y_speed))
    #     self.get_logger().info("Z Speed: {}".format(msg.sensor_str.z_speed))
    #     self.get_logger().info("Source Voltage: {}".format(msg.sensor_str.source_voltage))
    #     self.get_logger().info("Link Accelerometer X: {}".format(msg.sensor_str.link_accelerometer.x_data))
    #     self.get_logger().info("Link Accelerometer Y: {}".format(msg.sensor_str.link_accelerometer.y_data))
    #     self.get_logger().info("Link Accelerometer Z: {}".format(msg.sensor_str.link_accelerometer.z_data))
    #     self.get_logger().info("Link Gyroscope X: {}".format(msg.sensor_str.link_gyroscope.x_data))
    #     self.get_logger().info("Link Gyroscope Y: {}".format(msg.sensor_str.link_gyroscope.y_data))
    #     self.get_logger().info("Link Gyroscope Z: {}".format(msg.sensor_str.link_gyroscope.z_data))
    #     self.get_logger().info("End Flag: {}".format(msg.sensor_str.end_flag)) 
