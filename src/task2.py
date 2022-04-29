#! /usr/bin/python3
import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion

class GoToPose():
    def _init_(self):

        self.goal_sent = False

        # What to do if shut down (e.g. Ctrl-C or failure)
        rospy.on_shutdown(self.shutdown)
        
        # Tell the action client that we want to spin a thread by default
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        print("Wait for the action server to come up")

        # Allow up to 5 seconds for the action server to come up
        self.move_base.wait_for_server(rospy.Duration(5))

    def goto(self, pos, quat):
        # Send a goal
        self.goal_sent = True
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(Point(pos['x'], pos['y'], 0.000), Quaternion(quat['r1'], quat['r2'], quat['r3'], quat['r4']))

	# Start moving
        self.move_base.send_goal(goal)

	# Allow TurtleBot up to 60 seconds to complete task
        success = self.move_base.wait_for_result(rospy.Duration(60)) 

        state = self.move_base.get_state()
        result = False

        if success and state == GoalStatus.SUCCEEDED:
            # We made it!
            result = True
        else:
            self.move_base.cancel_goal()

        self.goal_sent = False
        return result

    def shutdown(self):
        if self.goal_sent:
            self.move_base.cancel_goal()
        # print("Stop")
        rospy.sleep(1)

    def go_to_point(self, position, quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000}):
        print("Go to ({}, {}) pose".format(position['x'], position['y']))
        success = self.goto(position, quaternion)

        if success:
            print("Hooray, reached the desired pose!")
        else:
            print("The base failed to reach the desired pose.")

if __name__ == '__main__':
    try:
        rospy.init_node('nav_test', anonymous=False)
        navigator = GoToPose()

        # Customize the following values so they are appropriate for your location
        position1 = {'x': 0.2, 'y' :0.2}
        quaternion1 = {'r1' : 1.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000}
        position2 = {'x': 0.2, 'y' : 0.4}
        quaternion2 = {'r1' : 1.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000}
        position3 = {'x': 0.2, 'y' : 0.1}
        quaternion3 = {'r1' : 1.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000}
        position4 = {'x': 0.3, 'y' : 0.2}
        quaternion4 = {'r1' : 1.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000}

        rospy.loginfo("Go to (%s, %s) pose1", position1['x'], position1['y'])
    
        success = navigator.goto(position3, quaternion3)
        if success:
            rospy.loginfo("Hooray, reached the desired pose1")
        else:
            rospy.loginfo("The base failed to reach the desired pose1")

        rospy.loginfo("Go to (%s, %s) pose2", position2['x'], position2['y'])
        success = navigator.goto(position2, quaternion2)

        if success:
            rospy.loginfo("Hooray, reached the desired pose2")
        else:
            rospy.loginfo("The base failed to reach the desired pose2")

        rospy.loginfo("Go to (%s, %s) pose3", position3['x'], position3['y'])
        success = navigator.goto(position3, quaternion3)

        if success:
            rospy.loginfo("Hooray, reached the desired pose3")
        else:
            rospy.loginfo("The base failed to reach the desired pose3")

        rospy.loginfo("Go to (%s, %s) pose4", position4['x'], position4['y'])
        success = navigator.goto(position4, quaternion4)

        if success:
            rospy.loginfo("Hooray, reached the desired pose4")
        else:
            rospy.loginfo("The base failed to reach the desired pose4")

        # Sleep to give the last log messages time to be sent
        rospy.sleep(1)

    except rospy.ROSInterruptException:
        rospy.loginfo("Ctrl-C caught. Quitting")