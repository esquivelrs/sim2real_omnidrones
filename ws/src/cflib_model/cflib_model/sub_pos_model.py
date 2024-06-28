import rclpy
from rclpy.node import Node

from std_msgs.msg import String

from motion_capture_tracking_interfaces.msg import NamedPoseArray
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import TransformStamped
import tf2_ros

import time
from threading import Thread

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.toc import TocFetcher, Toc
from cflib.crazyflie.mem import MemoryElement
from cflib.crazyflie.mem import Poly4D
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.utils import uri_helper
from geometry_msgs.msg import Pose
from scipy.spatial.transform import Rotation


# URI to the Crazyflie to connect to
uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')

# The name of the rigid body that represents the Crazyflie
rigid_body_name = 'cf231'

# True: send position and orientation; False: send position only
send_full_pose = True

# When using full pose, the estimator can be sensitive to noise in the orientation data when yaw is close to +/- 90
# degrees. If this is a problem, increase orientation_std_dev a bit. The default value in the firmware is 4.5e-3.
orientation_std_dev = 8.0e-3

position_estimate = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


class PosNode(Node):
    def __init__(self, cf):
        super().__init__('minimal_subscriber')
        custom_qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.subscription = self.create_subscription(
            NamedPoseArray,
            'poses',
            self.listener_callback,
            custom_qos_profile)
        self.subscription  # prevent unused variable warning
        self.cf = cf
        self.pose_publisher = self.create_publisher(Pose, 'state_estimate_pose', custom_qos_profile)
        self.br = tf2_ros.TransformBroadcaster(self)



    def listener_callback(self, msg):
        if msg.poses:
            for pose in msg.poses:
                if pose.name == rigid_body_name:
                    #self.get_logger().info(f'Pose name: {pose.name}, Position: x={pose.pose.position.x}, y={pose.pose.position.y}, z={pose.pose.position.z}')
                    #self.get_logger().info(f'Pose name: {pose.name}, Orientation: x={pose.pose.orientation.x}, y={pose.pose.orientation.y}, z={pose.pose.orientation.z}, w={pose.pose.orientation.w}')
                    send_extpose_quat(self.cf, pose.pose.position.x, pose.pose.position.y, pose.pose.position.z, pose.pose.orientation)

def wait_for_position_estimator(scf, pos_node):
    print('Waiting for estimator to find position...')
    

    log_config = LogConfig(name='Kalman Variance', period_in_ms=500)
    log_config.add_variable('kalman.varPX', 'float')
    log_config.add_variable('kalman.varPY', 'float')
    log_config.add_variable('kalman.varPZ', 'float')

    var_y_history = [1000] * 10
    var_x_history = [1000] * 10
    var_z_history = [1000] * 10

    threshold = 0.001

    with SyncLogger(scf, log_config) as logger:
        for log_entry in logger:
            
            rclpy.spin_once(pos_node, timeout_sec=0.1)
            data = log_entry[1]

            var_x_history.append(data['kalman.varPX'])
            var_x_history.pop(0)
            var_y_history.append(data['kalman.varPY'])
            var_y_history.pop(0)
            var_z_history.append(data['kalman.varPZ'])
            var_z_history.pop(0)

            min_x = min(var_x_history)
            max_x = max(var_x_history)
            min_y = min(var_y_history)
            max_y = max(var_y_history)
            min_z = min(var_z_history)
            max_z = max(var_z_history)

            # print("{} {} {}".
            #       format(max_x - min_x, max_y - min_y, max_z - min_z))

            if (max_x - min_x) < threshold and (
                    max_y - min_y) < threshold and (
                    max_z - min_z) < threshold:
                break


def send_extpose_quat(cf, x, y, z, quat):
    """
    Send the current Crazyflie X, Y, Z position and attitude as a quaternion.
    This is going to be forwarded to the Crazyflie's position estimator.
    """
    #print('Sending extpose: x={}, y={}, z={}, quat={}'.format(x, y, z, quat))
    if send_full_pose:
        cf.extpos.send_extpose(x, y, z, quat.x, quat.y, quat.z, quat.w)
    else:
        cf.extpos.send_extpos(x, y, z)


def reset_estimator(cf, pos_node):
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')

    # time.sleep(1)
    wait_for_position_estimator(cf, pos_node)


def adjust_orientation_sensitivity(cf):
    cf.param.set_value('locSrv.extQuatStdDev', orientation_std_dev)


def activate_kalman_estimator(cf):
    cf.param.set_value('stabilizer.estimator', '2')

    # Set the std deviation for the quaternion data pushed into the
    # kalman filter. The default value seems to be a bit too low.
    cf.param.set_value('locSrv.extQuatStdDev', 0.06)

def log_pos_callback(timestamp, data, logconf):
    print(data)
    global position_estimate
    position_estimate[0] = data['stateEstimate.x']
    position_estimate[1] = data['stateEstimate.y']
    position_estimate[2] = data['stateEstimate.z']
    
    quat = quaternion_from_euler(data['stateEstimate.roll'], data['stateEstimate.pitch'], data['stateEstimate.yaw'])
    position_estimate[3] = quat[0]
    position_estimate[4] = quat[1]
    position_estimate[5] = quat[2]
    position_estimate[6] = quat[3]
    
    #print('pos: ({}, {}, {})'.format(position_estimate[0], position_estimate[1], position_estimate[2]))
  
def publish_pose(pos_node):
    pose_msg = Pose()
    pose_msg.position.x = position_estimate[0]
    pose_msg.position.y = position_estimate[1]
    pose_msg.position.z = position_estimate[2]
    pose_msg.orientation.x = position_estimate[3]
    pose_msg.orientation.y = position_estimate[4]
    pose_msg.orientation.z = position_estimate[5]
    pose_msg.orientation.w = position_estimate[6]
                    
    # Use the mocap_wrapper instance to publish
    pos_node.pose_publisher.publish(pose_msg)
    
    t = TransformStamped()
    t.header.stamp = pos_node.get_clock().now().to_msg()
    t.header.frame_id = "world"  # Change to your reference frame
    t.child_frame_id = "estimate"  # Change to your object frame

    # Position
    t.transform.translation.x = position_estimate[0]
    t.transform.translation.y = position_estimate[1]
    t.transform.translation.z = position_estimate[2]

    # Orientation
    t.transform.rotation.x = position_estimate[3]
    t.transform.rotation.y = position_estimate[4]
    t.transform.rotation.z = position_estimate[5]
    t.transform.rotation.w = position_estimate[6]

    # Step 4: Publish the transform
    pos_node.br.sendTransform(t)
    

def quaternion_from_euler(roll: float, pitch: float, yaw: float):
    """Convert Euler angles to quaternion

    Args:
        roll (float): roll, in radians
        pitch (float): pitch, in radians
        yaw (float): yaw, in radians

    Returns:
        array: the quaternion [x, y, z, w]
    """
    return Rotation.from_euler(seq='xyz', angles=(roll, pitch, yaw), degrees=False).as_quat()  
    
def main(args=None):
    rclpy.init(args=args)
    cflib.crtp.init_drivers()

    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        cf = scf.cf
        pos_node = PosNode(cf)
        try:
            logconf = LogConfig(name='Position', period_in_ms=10)
            logconf.add_variable('stateEstimate.x', 'float')
            logconf.add_variable('stateEstimate.y', 'float')
            logconf.add_variable('stateEstimate.z', 'float')
            logconf.add_variable('stateEstimate.roll', 'float')
            logconf.add_variable('stateEstimate.pitch', 'float')
            logconf.add_variable('stateEstimate.yaw', 'float')
            
            adjust_orientation_sensitivity(cf)
            activate_kalman_estimator(cf)
            # duration = upload_trajectory(cf, trajectory_id, figure8)
            # print('The sequence is {:.1f} seconds long'.format(duration))
            reset_estimator(cf, pos_node)
            print('Estimator is reset')
            scf.cf.log.add_config(logconf)
            logconf.data_received_cb.add_callback(log_pos_callback)
            logconf.start()
            while rclpy.ok():
                # Spin once to check for incoming messages
                rclpy.spin_once(pos_node, timeout_sec=0.1)
                #print('pos: ({}, {}, {})'.format(position_estimate[0], position_estimate[1], position_estimate[2]))
                publish_pose(pos_node)
                
            
            logconf.stop()

            # You can add additional logic here that you want to run in the loop
        except KeyboardInterrupt:
            pass  # Handle Ctrl+C gracefully
        finally:
            # Cleanup and shutdown
            pos_node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()