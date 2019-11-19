import rospy
import smach, smach_ros


class Search(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes='end', 'found')
        

def main():
    
    rospy.init_node("approach")
    
    approach_sm = smach.StateMachine(outcomes = ['end', 'completed', 'failed'])
    
    with approach_sm:
        smach.StateMachine.add('Search', Search())

        smach.StateMachine.add('Move', Move())
    
    outcome = approach_sm.execute()
    rospy.spin()

if __name__ == "__main__":
    main()
