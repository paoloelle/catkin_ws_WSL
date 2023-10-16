#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped, Point, PoseArray
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from math import dist


current_state = State()

waypoint_index = 0
waypointList = []
waypointReceived = False # flag to check if waypoint list is received
nextWaypoint = [0,0,0]



def state_cb(msg):
    global current_state
    current_state = msg


#def waypointMATLAB_callback(data):
#
#    global waypointList
#    global waypointReceived
#    
#    for index in range(1, len(data.poses)):
#        waypointList.append([data.poses[index].position.x, data.poses[index].position.y, data.poses[index].position.z])
#    
#    #rospy.loginfo(str(waypointList))   
#
#    if waypointList:
#        waypointReceived = True
#        rospy.loginfo('Waypoint recived: ' + str(waypointReceived))


def buildWPArray(data):
    for index in range(1, len(data.poses)):
        waypointList.append([data.poses[index].position.x, data.poses[index].position.y, data.poses[index].position.z])


def getNextWP(currentPosition, threshold):

    global waypoint_index
    global waypointList
    global nextWaypoint

    try: 
        currentWaypoint = waypointList[waypoint_index]

        nextWaypoint = currentWaypoint


        if dist(currentPosition, currentWaypoint) < threshold: # compute euclidean 3D distance
            #if waypoint_index + 1 < len(waypointList):  
            waypoint_index += 1
            nextWaypoint = waypointList[waypoint_index]
            rospy.loginfo('Next waypoint: ' + str(nextWaypoint))             

    except IndexError:
        rospy.loginfo('No waypoint left')
    
    return nextWaypoint



def WP_callback(data):

    if waypointReceived:
        
        targetWP = getNextWP([data.pose.position.x,
                              data.pose.position.y,
                              data.pose.position.z], threshold=.2)          
        
        targetWP = [1, 1, 1]

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

    #else:
    #    rospy.loginfo('Waiting for waypoint')

if __name__ == '__main__':

    try:

        rospy.init_node('waypoint_manager')

        # subscribers
        state_sub = rospy.Subscriber("mavros/state", State, callback = state_cb)
        #getWP_sub = rospy.Subscriber("MATLAB_waypoint", PoseArray, callback=waypointMATLAB_callback)    
        position_sub = rospy.Subscriber('mavros/local_position/pose', PoseStamped, callback = WP_callback)

        
        # publisher
        currentWaypoint_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        
        

        
        rospy.wait_for_service("/mavros/cmd/arming")
        arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool) 


        rospy.wait_for_service("/mavros/set_mode")
        set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

        waypointMessage = rospy.wait_for_message("/MATLAB_waypoint", PoseArray)
        buildWPArray(waypointMessage)
        rospy.loginfo("WP list: " + str(waypointList))
        waypointReceived = True    

        # Setpoint publishing MUST be faster than 2Hz
        rate = rospy.Rate(20)

         # Wait for Flight Controller connection
        while(not rospy.is_shutdown() and not current_state.connected):
            rate.sleep()

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
    
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.logwarn("Node Interrupted")

