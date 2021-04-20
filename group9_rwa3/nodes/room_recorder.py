#! /usr/bin/env python

import rospy
from std_srvs.srv import Empty, EmptyRequest
from geometry_msgs.msg import PoseWithCovarianceStamped
from location_recorder.srv import RoomPose


def service_callback(request):
    """Write the position and orientation of the bot in the location_recorder.yaml file.

    The position and orientation values are called and used to create/edit the .yaml file.

    """
    request = str(request)[13:]
    request = request[:-1]
    positions = {'x: ': pos_x, 'y: ': pos_y, 'z: ': pos_z}
    orientations = {'x: ': orn_x, 'y: ': orn_y, 'z: ': orn_z, 'w: ': orn_w}
    file = open('/home/elliottmcg/catkin_ws1/src/group9_rwa3/records/location_recorder.yaml', 'a')
    file.write(request + ':' + '\n')
    space = ' '*4
    file.write(space + 'position:' + '\n')
    for key, value in positions.items():
        file.write(space + space + str(key) + str(value) + '\n')
    file.write(space + 'orientation:' + '\n')
    for key, value in orientations.items():
        file.write(space + space + str(key) + str(value) + '\n')
    return "Robot location saved."


def sub_callback(msg):
    """Get the position and orientation values.

    Reference /amcl_pose to get the position and orientation values.

    """

    global pos_x, pos_y, pos_z, orn_x, orn_y, orn_z, orn_w
    pos_x = msg.pose.pose.position.x
    pos_y = msg.pose.pose.position.y
    pos_z = msg.pose.pose.position.z
    orn_x = msg.pose.pose.orientation.x
    orn_y = msg.pose.pose.orientation.y
    orn_z = msg.pose.pose.orientation.z
    orn_w = msg.pose.pose.orientation.w
    return pos_x, pos_y, pos_z, orn_x, orn_y, orn_z, orn_w


if __name__ == "__main__":
    rospy.init_node('room_recorder')
    robot_pose = PoseWithCovarianceStamped()
    robot_pose = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, sub_callback)

    my_service = rospy.Service('/room_service', RoomPose, service_callback)
    rospy.wait_for_service('/room_service')
    get_pose_client = rospy.ServiceProxy('/room_service', Empty)
    get_pose_request_object = EmptyRequest()

    rospy.spin()
