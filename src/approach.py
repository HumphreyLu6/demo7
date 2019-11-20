import rospy
import smach, smach_ros
import actionlib
import tf
from ar_track_alvar_msgs.msg import AlvarMarker, AlvarMarkers
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist, Point, Quaternion

class Transformer():
    def __init__(self):
        self.listen = tf.TransformListener()
        self.br = tf.TransformBroadcaster()

    def build_new_frame(self, parent_frame_id, new_frame_id, relative_rotation, relative_transition):
        '''
        Params: parent_frame_id, new_frame_id, relative_rotation, relative_transition
        '''
        self.br.sendTransform(
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
        trans, rots = self.listen.lookupTransform(
            to_frame_id, from_frame_id, rospy.Time(0)
        )
        return trans, rots
            

class Search(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                            outcomes=['end', 'found'],
                            output_keys=['Search_out_tag_id'])
        
        self.ar_tag_id = None

    def execute(self, userdata):
        self.ar_pose_marker_sub = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.ar_tag_callback)
        while not rospy.is_shutdown():
            if self.ar_tag_id != None:
                self.ar_pose_marker_sub.unregister()
                userdata.Search_out_tag_id = self.ar_tag_id
                return 'found'
        self.ar_pose_marker_sub.unregister()
        return 'end'

    def ar_tag_callback(self, msg):
        if self.ar_tag_id == None and len(msg.markers) != 0:
            self.ar_tag_id = msg.markers[0].id

class Move(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                            outcomes = ['end', 'failed', 'arrived'],
                            input_keys = ['Move_in_tag_id'])
        self.transformer = Transformer()
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.client.wait_for_server()

    def execute(self, userdata):
        tag_id = userdata.Move_in__tag_id
        box_edge_length = 0.334
        distance_to_box_center = 0.5

        tag_box_transition = (0, 0, box_edge_length/2)
        tag_box_rotation = (0, 0, 1, 0)
        self.transformer.build_new_frame('ar_tag_'+ str(tag_id), 'box_ori_pose', tag_box_transition, tag_box_rotation)

        self.transformer.build_new_frame('box_ori_pose', 'box_front_park', (0, 0, distance_to_box_center), (0, 0, 0, 1)) #(0, 0, 0)
        self.transformer.build_new_frame('box_ori_pose', 'box_back_park', (0, 0, - distance_to_box_center), (0, 0, 1, 0)) #(0, 0, 180)
        self.transformer.build_new_frame('box_ori_pose', 'box_left_park', (0, - distance_to_box_center, 0), (0, 0, 0.707, 0.707)) #(0, 0, 90)
        self.transformer.build_new_frame('box_ori_pose', 'box_right_park', (0, distance_to_box_center, 0), (0, 0, -0.707, 0.707)) #(0, 0, 90)

        rospy.sleep(0.5)

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "box_front"
        goal.target_pose.pose.position = Point(0, 0, 0)
        goal.target_pose.pose.orientation = Quaternion(0, 0, 0, 1)
        self.client.send_goal(goal)
        self.client.wait_for_result()

        if rospy.is_shutdown():
            return 'end'
        return 'end'      

def main():
    
    rospy.init_node("approach")
    
    approach_sm = smach.StateMachine(
                    outcomes = ['end', 'completed', 'failed'],
                    output_keys = ['tag_id'])

    with approach_sm:
        smach.StateMachine.add('Search', Search(),
                                transitions = { 'end':'end',
                                                'found':'Move'},
                                remapping={'Search_out_tag_id':'tag_id'})

        smach.StateMachine.add('Move', Move(),
                                transitions = { 'end':'end',
                                                'failed':'failed',
                                                'arrived':'completed'},
                                remapping={'Move_in__tag_id':'tag_id'})
    
    outcome = approach_sm.execute()
    rospy.spin()

if __name__ == "__main__":
    main()
