#!/usr/bin/env python3
import sys
import rospy
from campusrover_msgs.msg import WorkingFloor
from campusrover_msgs.srv import RoutingPath
from nav_msgs.msg import Path
from campusrover_msgs.msg import LocationRoom
from campusrover_msgs.srv import PlannerFunction
from geometry_msgs.msg import Twist
from std_msgs.msg import String

location_info = WorkingFloor()
path_data = RoutingPath()
now_room = None
node_start_delay = None

map_pub = rospy.Publisher('working_floor', WorkingFloor, queue_size=10)
routing_client = rospy.ServiceProxy('generation_path', RoutingPath)
path_pub = rospy.Publisher('global_path',Path, queue_size=10)

def get_param():
    global location_info, now_room, node_start_delay
    # rospy.get_name()
    node_name = rospy.get_name()
    location_info.building = rospy.get_param(node_name + '/start_building','itc')
    location_info.floor = rospy.get_param(node_name + '/start_floor', '3')
    now_room = rospy.get_param(node_name + '/start_room', 'r314')
    node_start_delay = rospy.get_param(node_name + '/node_start_delay', 1.0)

def load_map():
    map_pub.publish(location_info)

def pub_routing_path(start_p, end_p):
    path_data = (routing_client(start_p,[end_p])).routing[0]
    path_pub.publish(path_data)

def navigation_callback(msg):

    print(now_room,'~~~~~~~~~~~',msg.data)
    pub_routing_path(now_room, msg.data) ######### define in srv

if __name__ == "__main__":
    rospy.init_node('teb_manager_node')

    get_param()

    rospy.Subscriber('navigation_pub', String, navigation_callback)

    rospy.sleep(node_start_delay)

    load_map()

    rospy.spin()
    