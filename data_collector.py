import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray
import csv
import os
from rclpy.qos import QoSProfile, ReliabilityPolicy


class DataCollector(Node):
    def __init__(self):
        super().__init__('data_collector')
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
            )
        self.subscription = self.create_subscription(
            Int16MultiArray,
            '/micro_ros_arduino_node_publisher',  # subscribe the topic
            self.listener_callback,
            qos_profile=qos_profile
            )
        self.csv_file_path = 'output.csv'
        self.message_count = 0  # initialize message count
        self.sampling_rate = 44100  # set sampling rate
        self.samples_per_msg = 128  # number of sample per message
        self.init_csv_file()
        
        # Create a timer that fires after 10 seconds to stop the node
        self.timer = self.create_timer(10, self.shutdown_callback)


    def init_csv_file(self):
        # Create new file every time we ran, clear previous data
        with open(self.csv_file_path, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['msg', 'time(second)'] + [f'data_{i}' for i in range(128)])  # set data size, the number of column


    def listener_callback(self, msg):
        self.message_count += 1  # message counter
        time_point = (self.message_count - 1) * self.samples_per_msg / self.sampling_rate  # calculate time
        # Map the incoming data from int16 values [-32768, 32767] to float values in mV
        # normalized_data = [(x * 500.0 / 32768.0) for x in msg.data]
        normalized_data = [(x * 0.01125) for x in msg.data]
        
        # Write data into CSV file
        with open(self.csv_file_path, mode='a', newline='') as file:
            writer = csv.writer(file)
            # timestamp = self.get_clock().now().to_msg()
            writer.writerow([self.message_count, time_point] + normalized_data)
            # writer.writerow([self.message_count, time_point] + list(msg.data))  # write original data
    
    def shutdown_callback(self):
        self.get_logger().info('Shutting down after 10 seconds.')
        rclpy.shutdown()
            

def main(args=None):
    rclpy.init(args=args)
    data_collector = DataCollector()
    rclpy.spin(data_collector)  # spin until shutdown
    data_collector.destroy_node()
    # rclpy.shutdown()

if __name__ == '__main__':
    main()

