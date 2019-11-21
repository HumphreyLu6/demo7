#!/usr/bin/env python
import rospy
import smach, smach_ros
import actionlib
import tf
from ar_track_alvar_msgs.msg import AlvarMarker, AlvarMarkers
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist, Point, Quaternion
from nav_msgs.msg import Odometry

import utils

class SearchParkTag(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes = ['end', 'found'])
        self.ar_tag_id = None
        self.transformer = utils.Transformer()
        self.timestamp = rospy.Time.now()

    def execute(self, userdata):
        self.ar_pose_marker_sub = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.ar_tag_callback)
        rospy.wait_for_message("/ar_pose_marker", AlvarMarkers)

        search_angle = {'middle': 0, 'left': 45, 'right': -45}

        while not rospy.is_shutdown():        
            for angle in search_angle:
                if rospy.is_shutdown():
                    return 'end'
                
                utils.rotate(search_angle[angle])
                rospy.sleep(0.1)
                utils.rotate(-search_angle[angle])
                if self.ar_tag_id != None:
                    return 'found'

        self.ar_pose_marker_sub.unregister()
        return 'end'
                

    def ar_tag_callback(self, msg):
        if self.ar_tag_id == None:
            for marker in msg.markers:
                if marker.id != 0:
                    self.ar_tag_id = marker.id
        else:
            temp_stamp = rospy.Time.now()
            if temp_stamp - self.timestamp > rospy.Duration(0.2):
                self.timestamp = temp_stamp

                self.transformer.build_new_frame('ar_marker_' + str(self.ar_tag_id), 'park_pose',
                                                 (0, 0, 0.3), (0, 0, 1, 0)
                                                )
            
            
class SearchBox(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                            outcomes=['end', 'found'])
        
        self.ar_tag_id = None
        self.transformer = utils.Transformer()
        self.timestamp = rospy.Time.now()

    def execute(self, userdata):
        self.ar_pose_marker_sub = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.ar_tag_callback)
        rospy.wait_for_message("/ar_pose_marker", AlvarMarkers)
        
        #utils.rotate(180)

        search_angle = {'middle': 0, 'left': 45, 'right': -45}

        while not rospy.is_shutdown():        
            for angle in search_angle:
                if rospy.is_shutdown():
                    return 'end'
                
                utils.rotate(search_angle[angle])
                print "go",angle, search_angle[angle]
                rospy.sleep(0.1)
                utils.rotate(-search_angle[angle])
                print "back", angle, -search_angle[angle]
                if self.ar_tag_id != None:
                    print "box_tag", self.ar_tag_id
                    return 'found'

        self.ar_pose_marker_sub.unregister()
        return 'end'

    def ar_tag_callback(self, msg):
        if self.ar_tag_id == None:
            for marker in msg.markers:
                if marker.id != 0:
                    self.ar_tag_id = marker.id
        else:
            temp_stamp = rospy.Time.now()
            if temp_stamp - self.timestamp > rospy.Duration(0.2):
                self.timestamp = temp_stamp

                box_edge_length = 0.334
                distance_to_box_center = 0.5

                tag_box_transition = (0, 0, box_edge_length/2)
                tag_box_rotation = (0, 0, 1, 0)
                self.transformer.build_new_frame('ar_marker_'+ str(self.ar_tag_id), 'box_center_pose',
                                                tag_box_transition, tag_box_rotation
                                                )

                self.transformer.build_new_frame('box_center_pose', 'box_front_park', 
                                                (0, 0, distance_to_box_center), (0, 0, 0, 1)
                                                ) #(0, 0, 0)
                self.transformer.build_new_frame('box_center_pose', 'box_back_park',
                                                (0, 0, - distance_to_box_center), (0, 0, 1, 0)
                                                ) #(0, 0, 180)
                self.transformer.build_new_frame('box_center_pose', 'box_left_park',
                                                (0, - distance_to_box_center, 0), (0, 0, 0.707, 0.707)
                                                ) #(0, 0, 90)
                self.transformer.build_new_frame('box_center_pose', 'box_right_park', 
                                                (0, distance_to_box_center, 0), (0, 0, -0.707, 0.707)
                                                ) #(0, 0, 90)
                #rospy.sleep(0.5)

class Approach(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                            outcomes = ['end', 'failed', 'arrived'])
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.client.wait_for_server()

    def execute(self, userdata):

        if rospy.is_shutdown():
            return 'end'

        utils.signal()
        #goal = utils.goal_pose('park_pose')
        while True:
            break
        goal = utils.goal_pose('box_front_park')
        self.client.send_goal(goal)
        result = self.client.wait_for_result(rospy.Duration.from_sec(0))
        utils.signal()

        if result == False:
            return 'failed'
        return 'arrived'   

class Push(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                            outcomes = ['end', 'completed'])
        self.odom = None

    def execute(self, userdata):
        self.transformer = utils.Transformer()
        trans, _ = self.transformer.look_up_transform('park_pose', 'box_center_pose')
        self.odom = {}
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.wait_for_message('/odom', Odometry)
        while not rospy.is_shutdown():
            print('Push:', trans)
        return 'end'
    
    def push_vertical(self, dist):
        start_odom = self.odom.copy()
        pass

    def push_horizontal(self,dist):
        pass
    
    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        self.odom['x'] = x
        self.odom['y'] = y


if __name__ == "__main__":

    rospy.init_node("demo7")
    approach_sm = smach.StateMachine(
                    outcomes = ['end', 'completed', 'failed'])

    with approach_sm:

        # smach.StateMachine.add('SearchParkTag', SearchParkTag(),
        #                         transitions = { 'end':'end',
        #                                         'found':'SearchBox'})

        smach.StateMachine.add('SearchBox', SearchBox(),
                                transitions = { 'end':'end',
                                                'found':'Approach'})

        smach.StateMachine.add('Approach', Approach(),
                                transitions = { 'end':'end',
                                                'failed':'failed',
                                                'arrived':'completed'})

        smach.StateMachine.add('Push', Push(),
                                transitions = { 'end':'end',
                                                'completed':'completed'})
    outcome = approach_sm.execute()
    rospy.spin()
