import rospy
# Brings in the SimpleActionClient
import actionlib
# Brings in the .action file and messages used by the move base action
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
import numpy as np

waypoints = [ 
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

def movebase_client(posearray):
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

# If the python node is executed as main process (sourced directly)
if __name__ == '__main__':
    pp = Pose_py()
    try:
        # Initializes a rospy node to let the SimpleActionClient publish and subscribe
        rospy.init_node('movebase_client_py')
        
        print("start navi")
        for i in range(len(waypoints)):
            print("start navi to", i+1, "goal")  
            result = movebase_client(waypoints[i])
            if result:
                rospy.loginfo("Goal execution done!")

        print("start finding aruco code")
        rospy.Subscriber("/aruco_single/pose",PoseStamped,pp.posecallback)
        pub = rospy.Publisher('/cmd_vel', Twist,queue_size=10)
        rate = rospy.Rate(10)
        
        msg=Twist()
        while not rospy.is_shutdown():
            if pp.aruco_z != 0 and abs(pp.aruco_x)<0.5:
                if pp.aruco_z>0.32:
                    ratio = (pp.aruco_z-0.20)
                    msg.angular.z=-8*(pp.aruco_x) 
                    msg.linear.x=max(0.2*(pp.aruco_z-0.20),0.05)
                    print("linear.x: ",msg.linear.x)
                    # print("ang.x: ",msg.angular.z)
                    pub.publish(msg)
                else:
                    print('stop')
                    for i in range(30):
                        rate.sleep()
                    msg.angular.z=0
                    msg.linear.x=0
                    pub.publish(msg)
                    pp.aruco_z=0.1
                    break

            else:
                print("searching")
                msg.angular.z = -0.35
                msg.linear.x = 0
                pub.publish(msg)
            
            rate.sleep()
        rospy.spin()


    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")