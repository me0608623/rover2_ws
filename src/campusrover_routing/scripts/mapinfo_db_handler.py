#!/usr/bin/env python3
import sys
import os

# 強制使用系統 Python 3.10 (避免 miniconda3 Python 3.13 衝突)
if sys.version_info.major >= 3 and sys.version_info.minor >= 13:
    # 當前 Python 是 3.13，重新執行使用 Python 3.10
        os.execv('/usr/bin/python3.10', [sys.executable] + sys.argv)

import math
import rclpy
from rclpy.node import Node
from campusrover_msgs.srv import ModuleInfo, MapLoadInfo, MapInitPose, MapClosestLocation
from campusrover_msgs.msg import NodeInfo, RoomConnected
from geometry_msgs.msg import Pose, Point, Quaternion, TransformStamped, Transform, Vector3, PoseStamped
from std_msgs.msg import Header

import requests
import json
import ast
import socket
import time

class MapInfoDbHandler(Node):
    def __init__(self):
        super().__init__('mapinfo_db_handler')
        self.get_parameters()
        self.map_info = {}

        self.srv_route_info = self.create_service(ModuleInfo, 'get_route_info', self.handle_get_route_info)
        self.srv_map_info = self.create_service(MapLoadInfo, 'get_map_info', self.handle_get_map_info)
        self.srv_init_point = self.create_service(MapInitPose, 'get_init_point', self.handle_get_init_point)
        self.srv_closest_location = self.create_service(MapClosestLocation, 'get_closest_location', self.handle_get_closest_location)

        self.check_connection()

    def get_parameters(self):
        self.use_database = self.declare_parameter('use_database', True).value
        self.json_folder = self.declare_parameter('json_folder', " ").value
        self.host = self.declare_parameter('host', '140.124.42.46').value
        self.port = self.declare_parameter('port', '8000').value

    def check_connection(self):
        if self.use_database:
            self.get_logger().info(f'Connecting to server {self.host}...')
            while not self.is_connected(self.host) and rclpy.ok():
                self.get_logger().info('Reconnecting to server...')
                time.sleep(2)
            self.get_logger().info('Server connected!')
        else:
            self.get_logger().info('使用離線 JSON 檔案...')

    def is_connected(self, hostname):
        try:
            host = socket.gethostbyname(hostname)
            s = socket.create_connection((host, int(self.port)), 2)
            s.close()
            return True
        except Exception:
            pass
        return False

    def is_updated(self, building, floor):
        keys = self.map_info.keys()
        if 'location' in keys and 'floor' in keys:
            return self.map_info['location'] == building and self.map_info['floor'] == floor
        return False

    def get_from_server(self, building, floor):
        # if not self.is_updated(building, floor):
        self.check_connection()
        if self.use_database:
            url = f'http://{self.host}:{self.port}/api/mapinfo/get_map_info/'
            payload = {'location': building, 'floor': floor}
            server_res = requests.get(url, params=payload, auth=('rover', 'campusrover314'))
            if server_res.status_code == 302:
                self.map_info = ast.literal_eval(server_res.text)
                return True
            else:
                self.get_logger().warn(f'No data is founded with parameter: {payload}')
                return False
        else:
            json_file_name = self.json_folder + f"{building}_{floor}f.json"
            try:
                with open(json_file_name, "r") as file:
                    self.map_info = json.load(file)
                    return True
            except Exception:
                self.get_logger().warn(f"No json data founded with {json_file_name}")
                return False
        # return True

    def handle_get_route_info(self, request, response):
        self.get_logger().info("handle_get_route_info called")
        if self.get_from_server(request.building, int(request.floor)):
            nodes = []
            connections = []

            for i in self.map_info['rooms']:
                m_pos = i['position']
                point = Point(x=m_pos['x'], y=m_pos['y'], z=m_pos['z'])
                orientation = Quaternion(x=m_pos['rx'], y=m_pos['ry'], z=m_pos['rz'], w=m_pos['rw'])
                node = NodeInfo(name=i['room'], pose=Pose(position=point, orientation=orientation))
                nodes.append(node)

            for conn in self.map_info['connections']:
                roomc = RoomConnected()
                roomc.connection = list(conn)
                connections.append(roomc)
            response.frame_id = 'map'
            response.node = nodes
            response.connections = connections
            return response
        else:
            # 填空欄位
            response.frame_id = ''
            response.node = []
            response.connections = []
            return response


    def handle_get_map_info(self, request, response):
        self.get_logger().info("handle_get_map_info called")
        if self.get_from_server(request.building, int(request.floor)):
            db_tf = self.map_info['tf']
            tf = Transform(
                translation=Vector3(x=db_tf['x'], y=db_tf['y'], z=db_tf['z']),
                rotation=Quaternion(x=db_tf['rx'], y=db_tf['ry'], z=db_tf['rz'], w=db_tf['rw'])
            )
            tf_stamped = TransformStamped(
                header=Header(stamp=self.get_clock().now().to_msg(), frame_id='world'),
                child_frame_id='map',
                transform=tf
            )
            response.tf = tf_stamped
            return response
        else:
            response.tf = TransformStamped()
            return response

    def handle_get_init_point(self, request, response):
        self.get_logger().info("handle_get_init_point called")
        if self.get_from_server(request.building, int(request.floor)):
            db_rooms = self.map_info['rooms']
            for room_obj in db_rooms:
                room = room_obj['room']
                pos = room_obj['position']
                if room == request.init_point:
                    header = Header(stamp=self.get_clock().now().to_msg(), frame_id='world')
                    position = Point(x=pos['x'], y=pos['y'], z=pos['z'])
                    orientation = Quaternion(x=pos['rx'], y=pos['ry'], z=pos['rz'], w=pos['rw'])
                    pose = Pose(position=position, orientation=orientation)
                    response.pose = PoseStamped(header=header, pose=pose)
                    return response
            response.pose = PoseStamped()
            return response
        else:
            response.pose = PoseStamped()
            return response

    def handle_get_closest_location(self, request, response):
        self.get_logger().info("handle_get_closest_location called")
        if not self.map_info:
            self.get_logger().warn("map is not load yet")
            response.closest_location = ""
            return response
        if request.building != self.map_info["location"] or int(request.floor) != self.map_info["floor"]:
            self.get_logger().warn("Location is not matched")
            response.closest_location = ""
            return response
        db_rooms = self.map_info['rooms']
        min_location = None
        for room_obj in db_rooms:
            label = room_obj['room']
            pos = room_obj['position']
            pt = Point(x=pos['x'], y=pos['y'], z=pos['z'])
            dis = math.sqrt(
                (request.robot_pose.pose.position.x - pt.x) ** 2 +
                (request.robot_pose.pose.position.y - pt.y) ** 2
            )
            if not min_location or dis < min_location[2]:
                min_location = (label, pt, dis)
        if min_location:
            response.closest_location = min_location[0]
        else:
            self.get_logger().warn("min_location is empty")
            response.closest_location = ""
        return response

def main(args=None):
    rclpy.init(args=args)
    node = MapInfoDbHandler()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
