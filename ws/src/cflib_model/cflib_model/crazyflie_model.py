import rclpy
from rclpy.node import Node

from std_msgs.msg import String

from motion_capture_tracking_interfaces.msg import NamedPoseArray
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import TransformStamped
import tf2_ros
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist

import time
from threading import Thread

from geometry_msgs.msg import Pose
from scipy.spatial.transform import Rotation
from crazyflie_interfaces.msg import LogDataGeneric

import numpy as np



from omegaconf import OmegaConf
import torch
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F
import numpy as np
import os
import sys
from tensordict.tensordict import TensorDict, TensorDictBase
from torchrl.data import (
    BinaryDiscreteTensorSpec,
    CompositeSpec,
    UnboundedContinuousTensorSpec,
    BoundedTensorSpec,
)




# The name of the rigid body that represents the Crazyflie
cf_name = 'cf231'




class CrazyflieSubscriber(Node):

    def __init__(self):
        super().__init__('crazyflie_subscriber')
        self.get_logger().info(f'Creating node with name: {cf_name}_subscriber')
        custom_qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.state = None
        self.pose = None 

        # send the cmd_vel to the crazyflie zeros
        self.publisher_ = self.create_publisher(Twist, f'/{cf_name}/cmd_vel_legacy', custom_qos_profile)


        self.init_cmd_vel()

    
        self.subscription_pose = self.create_subscription(PoseStamped,
                                                     f'/{cf_name}/pose',
                                                     self.pose_callback,
                                                     custom_qos_profile)
        
        #crazyflie_interfaces/msg/LogDataGeneric
        self.subscription_state = self.create_subscription(LogDataGeneric,
                                                        f'/{cf_name}/full_state',
                                                        self.state_callback,
                                                        custom_qos_profile)



        # self.subscription  # prevent unused variable warning

    def pose_callback(self, msg):
        #self.get_logger().info('I heard: "%s"' % msg.pose)
        #create a numpy array from the pose
        self.pose = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
        self.get_action()
    
    def state_callback(self, msg):
        #self.get_logger().info('I heard: "%s"' % msg.values)
        # array('f', [44.0, -389.0, 60.0, 6.0, 0.0, -1.0, 9.0, -109.0, 9839.0])
        # vx, vy, vz, rateRoll, ratePitch, rateYaw, ax, ay, az
        #
        #create a numpy array from the values
        self.state = np.array(msg.values)

    def init_cmd_vel(self):
        # create a Twist message
        for i in range(10):
            twist = Twist()
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = 0.0
            self.publisher_.publish(twist)
            # sleep for 0.1 seconds
            time.sleep(0.1)

    def get_action(self):
        if self.pose is not None and self.state is not None:
            print(f'Pose: {self.pose} \n State: {self.state}')

            # create a Twist message
            twist = Twist()
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = 10000.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = 0.0
            self.publisher_.publish(twist)

            
            # self.pose = None

        else:
            self.get_logger().info('Pose or state is None')
            self.get_logger().info(f'Pose: {self.pose} \n State: {self.state}')



def main(args=None):
    rclpy.init(args=args)
    # Create the node


    cf_subs = CrazyflieSubscriber()



    rclpy.spin(cf_subs)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    cf_subs.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()