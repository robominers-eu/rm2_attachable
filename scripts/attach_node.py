#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String
from rm2_attachable.action import AttachModel
import time
from rclpy.action import ActionClient

class AttachController(Node):
    def __init__(self):
        super().__init__('attach_controller')
        self.contact_publisher = self.create_publisher(String, "/AttacherContact/contact" ,10)
        self.vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.touched_subscriber = self.create_subscription(Bool, "/AttacherContact/touched", self.touched_callback, 10)

        self.action_client = ActionClient(self, AttachModel, 'AttachableJoint') 

        self.publish_contact()

    def call_action(self):
        goal_msg = AttachModel.Goal() 
        # TODO GOAL MSG
        future = self.action_client.send_goal_async(goal_msg)

        # Wait for the result
        rclpy.spin_until_future_complete(self, future)
        result = future.result()

        if result.status == AttachModel.Result.SUCCESS:
            self.get_logger().info('Action succeeded!')
            return True
        else:
            self.get_logger().info('Action failed!')
            return False        

    def publish_contact(self):
        msg = String()
        msg.data = "[rm2_sim][couplingLink][rm2_sim_mining][base_link]"
        self.contact_publisher.publish(msg)
        for _ in range(3):
            self.publish_velocity()
        

    def publish_velocity(self, velocity = 0.8):
        msg = Twist()
        msg.linear.x = velocity
        self.vel_publisher.publish(msg)

    def touched_callback(self, msg):
        if msg.data: # robots in contact
            # self.get_logger().info("CONTACT")
            result = self.call_action()
            if result:
                self.publish_velocity(-0.8)
                self.wait()
                self.publish_velocity(0.8)
                self.wait()
                self.publish_velocity(0.0)
                self.get_logger().info("----------- COMPLETING MINING TASK -----------")
    
    def wait(self, duration = 4.0):
        start_time = time.time()
        while (time.time() - start_time) < duration:
            time.sleep(0.1)  # Adjust the sleep duration if needed





def main(args=None):
    rclpy.init(args=args)
    robot_controller = AttachController()
    rclpy.spin(robot_controller)
    robot_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()