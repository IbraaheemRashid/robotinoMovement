import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import socket
import struct

class Robotino3BatteryHAL(Node):
    def __init__(self):
        super().__init__('robotino3_battery_hal')
        
        self.robotino_ip = '10.42.0.232'  
        self.robotino_port = 30001
        self.socket = None
        
        self.battery_pub = self.create_publisher(
            Float32, 
            'robotino/battery_voltage', 
            10
        )
        
        self.timer = self.create_timer(1.0, self.read_battery)
        
        self.connect_to_robotino()
        
    def connect_to_robotino(self):
        """Establish connection to Robotino"""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.connect((self.robotino_ip, self.robotino_port))
            self.get_logger().info(f'Connected to Robotino at {self.robotino_ip}:{self.robotino_port}')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to Robotino: {str(e)}')
            
    def read_battery(self):
        """Read battery voltage and publish to ROS topic"""
        if not self.socket:
            self.get_logger().warn('No connection to Robotino, attempting to reconnect...')
            self.connect_to_robotino()
            return
            
        try:
            command = b'\x02\x01\x00\x00\x00\x00\x00\x00'
            self.socket.send(command)
            
            response = self.socket.recv(8)
            if len(response) == 8:
                voltage = struct.unpack('!f', response[4:])[0]
                
                msg = Float32()
                msg.data = voltage
                self.battery_pub.publish(msg)
                
                self.get_logger().info(f'Battery voltage: {voltage:.2f}V')
            else:
                self.get_logger().warn('Received incomplete data from Robotino')
                
        except Exception as e:
            self.get_logger().error(f'Failed to read battery status: {str(e)}')
            self.socket = None
            
    def cleanup(self):
        """Clean up resources"""
        if self.socket:
            try:
                self.socket.close()
            except:
                pass

def main(args=None):
    rclpy.init(args=args)
    node = Robotino3BatteryHAL()
    
    try:
        print("connected")
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()