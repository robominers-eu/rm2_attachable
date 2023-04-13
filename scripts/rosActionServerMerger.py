#!/usr/bin/env python3
from mimetypes import init
from re import S
import time
from urllib import request

from numpy import empty

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from rm2_attachable.action import AttachModel
from std_msgs.msg import String
from std_msgs.msg import Int32
from std_msgs.msg import Empty
from std_msgs.msg import Bool


from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter
from rcl_interfaces.msg import ParameterValue

class AttachableJointActionServer(Node):

    def __init__(self):
        self.contact = False
        super().__init__('AttachableJointActionServer')
        self._action_server = ActionServer(
            self,
            AttachModel,
            'AttachableJoint',
            self.execute_callback)

        self.contactPublisherTopic = "/AttacherContact/contact"
        self.contactSubscriberTopic = "/AttacherContact/touched"
        self.contactPublisher = self.create_publisher(String, self.contactPublisherTopic,10)
        self.contactSubscriber = self.create_subscription(Bool, self.contactSubscriberTopic, self.contactSubscriber_callack, 10)
        
        self.attachableJointErrorSubscriber = self.create_subscription(Int32, self.contactSubscriberTopic, self.errorSubscriber_callack, 10)

        self.attachableJointPublisher = self.create_publisher(String, "/AttachableJoint" ,10)

        self.msgToPublish = String()

        self.error = 0


        
    def contactSubscriber_callack(self, msg):
        self.contact = msg.data        


    def errorSubscriber_callack(self, msg):
        self.error = msg.data        

    def waitToResponse(self, timeout):
        initTime = time.time()
        while rclpy.ok():
            rclpy.spin_once(self)
            if ((time.time() - initTime) > timeout):
                break

    def attachModelIgnition(self, request):
        
        self.msgToPublish.data = '[{}][{}][{}][{}][{}]'.format(self.parentModel, self.parentLink, self.childModel, self.childLink, request)
        self.attachableJointPublisher.publish(self.msgToPublish)
        
    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        
        result = AttachModel.Result()

        feedback_msg = AttachModel.Feedback()
        self.childLink = goal_handle.request.child_link
        self.parentLink = goal_handle.request.parent_link
        self.childModel = goal_handle.request.child_model
        self.parentModel = goal_handle.request.parent_model

        self.get_logger().info("Executing Goal...")
        if True == goal_handle.request.attach:
            #Check if the linsk are touching       
            self.msgToPublish.data = '[{}][{}][{}][{}]'.format(self.parentModel, self.parentLink, self.childModel, self.childLink)

            self.contactPublisher.publish(self.msgToPublish)
            
            self.waitToResponse(0.5)

            self.msgToPublish.data = 'end'
            self.contactPublisher.publish(self.msgToPublish)

            if self.contact:
                self.get_logger().info("Links are in contact, Attaching models...")
                #Add model in Gazebo Ingition
                self.attachModelIgnition("attach")
                # self.waitToResponse(0.5)
                feedback_msg = self.error
                if (self.error == 0):
                    result.response = "True"
            else:
                self.get_logger().info("Links are NOT in contact. End.")
                result.response = "False"

        else:
            #Detach Models in ignition
            self.get_logger().info("Detach models...")
            self.attachModelIgnition("detach")
            # self.waitToResponse(0.5)
            feedback_msg = self.error
            if (self.error == 0):
                result.response = "True"

        result.response = "Contact Done: " + result.response + " || Error: " + str(self.error)
        goal_handle.succeed()        
        return result

def main(args=None):
    rclpy.init(args=args)
    attachable_joint_action_server = AttachableJointActionServer()
    while rclpy.ok():
        rclpy.spin_once(attachable_joint_action_server)
    

if __name__ == '__main__':
    main()

