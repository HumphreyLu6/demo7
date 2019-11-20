#!/usr/bin/env python
import rospy
import numpy
import tf
from enum import Enum
from geometry_msgs.msg import Twist, Point, Quaternion
from nav_msgs.msg import Odometry
from rospy import ROSException
from ros_numpy import numpify
from tf.transformations import decompose_matrix
from kobuki_msgs.msg import Led, Sound
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

twist_pub = rospy.Publisher("/cmd_vel_mux/input/teleop", Twist, queue_size=1)
led_pub_1 = rospy.Publisher('/mobile_base/commands/led1', Led, queue_size=1)
led_pub_2 = rospy.Publisher('/mobile_base/commands/led2', Led, queue_size=1)
sound_pub = rospy.Publisher('/mobile_base/commands/sound', Sound, queue_size=1)

def approxEqual(a, b, tol = 0.001):
    return abs(a - b) <= tol

def signal(quantity=1,onColor=Led.GREEN,offColor=Led.BLACK,interval=0.5):
    
    if quantity == 1:
        led_pub_1.publish(onColor)
    elif quantity == 2:
        led_pub_2.publish(onColor)
    else:
        led_pub_1.publish(onColor)
        led_pub_2.publish(onColor)

    for i in range(quantity):
        sound_pub.publish(1)
        rospy.sleep(interval)

    led_pub_1.publish(offColor)
    led_pub_2.publish(offColor)

def wait_for_odom_angle(timeout=None):
    odom = rospy.wait_for_message("odom", Odometry, timeout=timeout)
    pose = numpify(odom.pose.pose)
    _, _, angles, _, _ = decompose_matrix(pose)
    theta = angles[2] * 180 / 3.14159
    return theta

def goal_pose(frame_id, point = Point(0, 0, 0), quaternion = Quaternion(0, 0, 0, 1)):  
    '''
    Params: frame_id, point = Point(0, 0, 0), quaternion = Quaternion(0, 0, 0, 1)
    Return: goal_pose
    '''  
    goal_pose = MoveBaseGoal()
    goal_pose.target_pose.pose.position = point
    goal_pose.target_pose.pose.orientation = quaternion
    return goal_pose
    
def rotate(angle=90, max_error=3, anglular_scale=1.0):
    '''
    input: angle=90, max_error=3, anglular_scale=1.0
    '''

    init_theta = wait_for_odom_angle()
    theta = init_theta
    direction = numpy.sign(angle)
    target_theta = init_theta + angle
    if target_theta > 180:
        target_theta = target_theta % 360 - 360
    if target_theta < -180:
        target_theta = target_theta % -360 + 360

    while abs(target_theta - theta) > max_error:
        out_twist = Twist()
        out_twist.angular.z = direction * anglular_scale
        twist_pub.publish(out_twist)
        theta = wait_for_odom_angle()
    twist_pub.publish(Twist())



class Transformer():
    def __init__(self):
        self.__listen = tf.TransformListener()
        self.__br = tf.TransformBroadcaster()

    def build_new_frame(self, parent_frame_id, new_frame_id, relative_rotation, relative_transition):
        '''
        Params: parent_frame_id, new_frame_id, relative_rotation, relative_transition
        '''
        self.__br.sendTransform(
            relative_transition,
            relative_rotation,
            rospy.Time.now(),
            new_frame_id,
            parent_frame_id,
        )

    def look_up_transform(self, from_frame_id, to_frame_id, time = rospy.Time(0)):
        '''
        Params: from_frame_id, to_frame_id, time = rospy.Time(0)
        Returns: (trans, rot) from from_frame_id to to_frame_id
        '''
        trans, rots = self.__listen.lookupTransform(
            to_frame_id, from_frame_id, rospy.Time(0)
        )
        return trans, rots

if __name__ == "__main__":
    rospy.init_node("Util_Test")
    while not rospy.is_shutdown():
        rotate(90,anglular_scale=1)
        signal(1, onColor=Led.ORANGE)
        rotate(-180,anglular_scale=2)
        signal(2,onColor=Led.RED)
        rotate(180, anglular_scale=2)
        signal(3,interval=1)
        quit()