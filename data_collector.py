import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray
import csv
import os
from rclpy.qos import QoSProfile, ReliabilityPolicy
from datetime import datetime, timedelta
from std_msgs.msg import Bool


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
        self.run_time = 20             # Define the runtime for recording
        self.sampling_rate = 44100    # Set sampling rate
        self.samples_per_msg = 128    # number of sample per message
        self.message_count = 0        # Initialize message count
        self.init_csv_file()
        self.start_time = self.get_clock().now()  # Record the start time
        
        # Create a timer that fires after 10 seconds to stop the node
        self.timer = self.create_timer(self.run_time, self.shutdown_callback)
   
    def init_csv_file(self):
        # Create new file every time we ran, clear previous data
        with open(self.csv_file_path, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['msg', 'time(second)', 'tonesweep_freq'] + [f'data_{i}' for i in range(128)])  # set data size, the number of column
            
    def listener_callback(self, msg):
        self.message_count += 1  # message counter
        current_time = self.get_clock().now()
        elapsed_time = current_time - self.start_time  # Get elapsed time from the start
        elapsed_seconds = elapsed_time.nanoseconds / 1e9  # Convert nanoseconds to seconds

        # Map the incoming data from int16 values [-32768, 32767] to float values in mV
        # normalized_data = [(x * 0.00425453) for x in msg.data[1:]]   # before amplified, LineIN:15
        normalized_data = [(x * 0.15) for x in msg.data[1:]]
        
        # Write data into CSV file
        with open(self.csv_file_path, mode='a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([self.message_count, elapsed_seconds, msg.data[0]] + normalized_data)
            # writer.writerow([self.message_count, time_point] + list(msg.data))  # write original data    
    
    def shutdown_callback(self):
        self.get_logger().info(f'Shutting down after {self.run_time} seconds.')
        rclpy.shutdown()
            

def main(args=None):
    rclpy.init(args=args)
    data_collector = DataCollector()
    rclpy.spin(data_collector)  # spin until shutdown
    data_collector.destroy_node()
    # rclpy.shutdown()

if __name__ == '__main__':
    main()

