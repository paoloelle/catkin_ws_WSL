#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from math import dist
from scipy.io import loadmat

current_state = State()

waypoint_index = 0
waypointList = [
   [0,   0,    1],
   [0,   1,    1],
   [0,   2,    1],
   [0,   3,    1],
   [1,   0,    1],
   [1,   1,    1],
   [1,   2,    1],
   [1,   3,    1],
   [2,   0,    1],
   [2,   1,    1],
   [2,   2,    1],
   [2,   3,    1],
   [3,   0,    1],
   [3,   1,    1],
   [3,   2,    1],
   [3,   3,    1],
   [4,   0,    1],
   [4,   1,    1],
   [4,   2,    1],
   [4,   3,    1],
   [5,   0,    1],
   [5,   1,    1],
   [5,   2,    1],
   [5,   3,    1],
   [6,   0,    1],
   [6,   1,    1],
   [6,   2,    1],
   [6,   3,    1],
   [7,   0,    1],
   [7,   1,    1],
   [7,   2,    1],
   [7,   3,    1]
    ]



def state_cb(msg):
    global current_state
    current_state = msg



def getNextWP(currentPosition, threshold):

    global waypoint_index
    global waypointList

    currentWaypoint = waypointList[waypoint_index]

    nextWaypoint = currentWaypoint
    
    try:
        if dist(currentPosition, currentWaypoint) < threshold:
            if waypoint_index + 1 < len(waypointList):  
                waypoint_index += 1              
                nextWaypoint = waypointList[waypoint_index]
                rospy.loginfo('Next waypoint: ' + str(nextWaypoint))             

    except ValueError:
        rospy.logwarn('No waypoint left')

    
    return nextWaypoint



def WP_callback(data):
    
    targetWP = getNextWP([data.pose.position.x,
                          data.pose.position.y,
                          data.pose.position.z], threshold=.2)          

    # Create a PoseStamped message
    pose_msg = PoseStamped()
    pose_msg.header.stamp = rospy.Time.now()
    pose_msg.pose.position.x = targetWP[0]
    pose_msg.pose.position.y = targetWP[1]
    pose_msg.pose.position.z = targetWP[2]
    pose_msg.pose.orientation.x = 0.0
    pose_msg.pose.orientation.y = 0.0
    pose_msg.pose.orientation.z = 0.0
    pose_msg.pose.orientation.w = 0.0

    currentWaypoint_pub.publish(pose_msg)

if __name__ == '__main__':

    try:

        rospy.init_node('waypoint_manager')

        # subscribers
        state_sub = rospy.Subscriber("mavros/state", State, callback = state_cb)
        position_sub = rospy.Subscriber('mavros/local_position/pose', PoseStamped, callback = WP_callback)

        # publisher
        currentWaypoint_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)

        rospy.wait_for_service("/mavros/cmd/arming")
        arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)

        rospy.wait_for_service("/mavros/set_mode")
        set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)


        # Setpoint publishing MUST be faster than 2Hz
        rate = rospy.Rate(20)

        offb_set_mode = SetModeRequest()
        offb_set_mode.custom_mode = 'OFFBOARD'

        arm_cmd = CommandBoolRequest()
        arm_cmd.value = True

        last_req = rospy.Time.now()

        while(not rospy.is_shutdown()):
            if(current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                if(set_mode_client.call(offb_set_mode).mode_sent == True):
                    rospy.loginfo("OFFBOARD enabled")

                last_req = rospy.Time.now()
            else:
                if(not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                    if(arming_client.call(arm_cmd).success == True):
                        rospy.loginfo("Vehicle armed")

                    last_req = rospy.Time.now()

            rate.sleep()

    except rospy.ROSInterruptException:
        pass

