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
        self.get_logger().info('Executing attach action')
        self.contact_publisher = self.create_publisher(String, "/AttacherContact/contact" ,10)
        self.vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.touched_subscriber = self.create_subscription(Bool, "/AttacherContact/touched", self.touched_callback, 10)
        self.action_client = ActionClient(self, AttachModel, 'AttachableJoint') 
        self.publish_contact()
        
        self.wait()
        self.call_action()
    def call_action(self):
            goal_msg = AttachModel.Goal()

            goal_msg.parent_model = "rm2_sim"
            goal_msg.parent_link = "couplingLink"
            goal_msg.child_model = "rm2_sim_mining"
            goal_msg.child_link = "base_link"
            goal_msg.attach = True
            self.action_client.wait_for_server()
            self.future = self.action_client.send_goal_async(goal_msg)
            self.future.add_done_callback(self.goal_response_callback)


    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result

        if result.response == "True":
            self.get_logger().info('Action succeeded!')
            self.publish_velocity(-7.0)
            self.wait(3.0)
            self.publish_velocity(5.0)
            self.wait(5.0)
            self.publish_velocity(0.0)
            self.get_logger().info("----------- RESUMING TO MINING TASK -----------")      

        else:
            self.get_logger().info('Action failed!')
            rclpy.shutdown()
        

    def publish_contact(self):
        msg = String()
        msg.data = "[rm2_sim][couplingLink][rm2_sim_mining][base_link]"
        self.contact_publisher.publish(msg)
        for _ in range(3):
            self.publish_velocity()
            self.wait(1.0)


    def publish_velocity(self, velocity = 0.8):
        msg = Twist()
        msg.linear.x = velocity
        self.vel_publisher.publish(msg)
        

    def touched_callback(self, msg):
            self.publish_velocity(0.2)
            if msg.data: # robots in contact
                self.publish_velocity(0.2)


    
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