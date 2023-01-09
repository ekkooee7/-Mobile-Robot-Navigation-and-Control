#!/usr/bin/env python
import roslib
import rospy
import smach
import smach_ros
# Brings in the .action file and messages used by the move base action
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
import numpy as np
import actionlib


# 没用了
# class Foo(smach.State):
#    def __init__(self):
#        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
#        self.counter = 0
#    def execute(self, userdata):
#        rospy.loginfo('Executing state FOO')
#        if self.counter < 3:
#            self.counter += 1
#            return 'outcome1'
#        else:
#            return 'outcome2'

# define state Bar
class Bar(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome_navi'])

    def execute(self, userdata):
        rospy.loginfo('Executing state BAR')
        return 'outcome_navi'

class Navi(smach.State):
    def __init__(self):
        self.waypoints = [ 
            [(3.62,1.0377,0),(0.0,0.0,0.961,0.27)], 
            [(3.08,1.1093, 0),(0.0,0.97,0,179)],
            [(0.76,2.84,0),(0.0,0.0,-0.72,0.68)],
            [(1.765,-0.19,0),(0.0,0.0,0.-0.656,0.753)],
            [(3.313, -2.289, 0), (0.0, 0.0, -0.785, 0.6199)],
            [(-0.498, -0.934, 0), (0, 0, 0.97, 0.243)],
            [(2.3781,1.4101,0),(0,0,-0.23,0.9717)],
            [(4.6585,1.5379,0),(0,0,0.0998,-0.04356)], 
            [(4.1391,1.3852,0),(0,0,0.9,0.08)]
            ]
        smach.State.__init__(self, outcomes=['outcome_navi','outcome_aruco'])
        self.cnt = 0
    def execute(self, ud):
        if self.cnt < len(self.waypoints):
            info_msg = "Start navigating to " + str(self.cnt) + "th point"
            rospy.loginfo(info_msg)
            # print(self.waypoints[self.cnt])
            result = self.movebase_client(self.waypoints[self.cnt])
            if result:
                rospy.loginfo("Goal execution done!")
            self.cnt += 1
            return 'outcome_navi'
        else:
            rospy.loginfo('Navigation completed')
            return 'outcome_aruco'
    
    def movebase_client(self, posearray):
        # Create an action client called "move_base" with action definition file "MoveBaseAction"
        client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
 
        # Waits until the action server has started up and started listening for goals.
        client.wait_for_server()
   
        # Creates a new goal with the MoveBaseGoal constructor
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        # Move 0.5 meters forward along the x axis of the "map" coordinate frame 
        goal.target_pose.pose.position.x = posearray[0][0]
        goal.target_pose.pose.position.y = posearray[0][1]
        goal.target_pose.pose.position.z = posearray[0][2]
        # No rotation of the mobile base frame w.r.t. map frame
        goal.target_pose.pose.orientation.w = posearray[1][3]
    
        # Sends the goal to the action server.
        client.send_goal(goal)
 
        # Waits for the server to finish performing the action.
        wait = client.wait_for_result()
        # If the result doesn't arrive, assume the Server is not available
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
        # Result of executing the action
            return client.get_result()  

class Pose_py():
    def __init__(self):
        self.aruco_x = 0
        self.aruco_y = 0
        self.aruco_z = 0
    def posecallback(self, data):
        self.aruco_z=data.pose.position.z
        self.aruco_x=data.pose.position.x

#define state ArucoFinder
class ArucoFinder(smach.State):
    def __init__(self):
        print("start finding aruco code")
        smach.State.__init__(self, outcomes=['outcome_navi','outcome_aruco'])
        rospy.Subscriber("/aruco_single/pose",PoseStamped,pp.posecallback)
        pub = rospy.Publisher('/cmd_vel', Twist,queue_size=10)
        msg=Twist()
        pp = Pose_py()
        findresult=False

    def execute(self, ud):
        while (not rospy.is_shutdown()) and not self.findresult:
            if self.pp.aruco_z != 0 and abs(self.pp.aruco_x)<0.5:
                if self.pp.aruco_z>0.32:
                    ratio = (self.pp.aruco_z-0.20)
                    self.msg.angular.z=-8*(self.pp.aruco_x) 
                    self.msg.linear.x=max(0.2*(self.pp.aruco_z-0.20),0.05)
                    print("linear.x: ",self.msg.linear.x)
                    self.pub.publish(self.msg)
                else:
                    print('stop')
                    for i in range(30):
                        self.rate.sleep()
                    self.msg.angular.z=0
                    self.msg.linear.x=0
                    self.pub.publish(self.msg)
                    self.findresult=True
            else:
                print("searching")
                self.msg.angular.z = -0.35
                self.msg.linear.x = 0
                self.pub.publish(self.msg)
            self.rate.sleep()
            rospy.spin()
        self.findresult=False
        return 'FindAruco'

if __name__ == '__main__':
    rospy.init_node('lab7')
    # Create a SMACH state machine 
    sm = smach.StateMachine(outcomes=['outcome_end']) # 設定結束狀態為outcome_end
    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('Navi',Navi(),
                                transitions={'outcome_navi':'BAR', 'outcome_aruco':'Arucofinder'})
        smach.StateMachine.add('Arucofinder', ArucoFinder(), 
                               transitions={'FindAruco':'FOO'})
        smach.StateMachine.add('BAR', Bar(),    
                               transitions={'outcome_navi':'Navi'})

    # Execute SMACH plan
    outcome = sm.execute()

