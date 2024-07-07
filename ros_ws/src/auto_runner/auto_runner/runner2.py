import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.time import Time
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import TwistStamped, Twist, Vector3
from yolov8_msgs.srv import CmdMsg
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from auto_runner.map_transform import convert_map
from auto_runner.lib.parts import *
from auto_runner.lib.car_drive2 import RobotController2
from auto_runner.lib.common import Message, Observable, SearchEndException


class GridMap(Node):
    def __init__(self) -> None:
        super().__init__("grid_map_node")

        # 기본 콜백 그룹 생성
        self._default_callback_group = ReentrantCallbackGroup()

        # 맵 수신 10*10
        self.map_pub = self.create_subscription(
            OccupancyGrid,
            "/occ_grid_map",
            callback=self.receive_map,
            callback_group=self._default_callback_group,
            qos_profile=10,
        )

    def receive_map(self, map: OccupancyGrid):
        raw_data = np.array(map.data, dtype=np.int8)
        grid_map = convert_map(raw_data)
        MapData.publish_(data=grid_map)


class LidarScanNode(Node):
    def __init__(self) -> None:
        super().__init__("lidar_scan_node")

        # 기본 콜백 그룹 생성
        self._default_callback_group = ReentrantCallbackGroup()

        # lidar subscriber
        self.create_subscription(
            LaserScan,
            "jetauto/scan",
            callback=self.scan_handler,
            callback_group=self._default_callback_group,
            qos_profile=10,
        )

        self.get_logger().info(f"LidarScanNode started...")

    def scan_handler(self, msg: LaserScan):
        LidarData.publish_(data=msg)


class AStartSearchNode(Node):

    def __init__(self) -> None:
        super().__init__("path_search_astar")
        queue_size = 1
        self.twist_msg = Twist()

        # odom 위치정보 취득
        qos_profile = QoSProfile(depth=queue_size)
        self.create_subscription(
            TwistStamped,
            "/unity_tf",
            callback=self.unity_tf_callback,
            qos_profile=qos_profile,
        )
        self.pub_jetauto_car = self.create_publisher(
            Twist, "jetauto_car/cmd_vel", queue_size
        )

        self.srv_jetauto_car = self.create_service(
            CmdMsg, "jetauto_car/cmd_msg", self.send_command
        )

        Observable.subscribe(o=self.update, subject="node")
        LidarData.subscribe(o=self.laser_scan)
        self.setup()

        self.get_logger().info(f"AStart_Path_Search_Mode has started...")

    def setup(self) -> None:
        self.robot_ctrl = RobotController2(node=self)
        self.step_completed = True
        self.is_searching_finished = False
        self.nanoseconds = 0

        self.get_logger().info("초기화 처리 완료")

    def unity_tf_callback(
        self, imu_data: TwistStamped
    ) -> None:  # msg로부터 위치정보를 추출

        IMUData.publish_(data=imu_data)

        if self.is_searching_finished:
            return
        # 로봇 이동계획을 수립한다.
        # 다음 위치는 무엇이고 회전인지 직진인지 아니면 후진을 해야하는지를 판단해야함
        # 계획이 세워지면, thread를 통해 추진하고 경과를 event를 통해 전달한다.
        if self.step_completed:
            if not self.robot_ctrl.prepare():
                self.print_log("search Finished.....")
                self.is_searching_finished = True
                return
            action_plan = self.robot_ctrl.make_plan()

            # 로봇에 계획을 전달한다.
            self.robot_ctrl.execute(action_plan)
            self.step_completed = False

        else:
            # 각종 체크 수행
            # IMU/lidar를 통해 장애물, 경로이탈, 동체수평 여부 체크를 한다.
            if self._is_near():
                self._send_message(
                    title="근접제어", x=self.state_near[0], theta=self.state_near[1]
                )

    # 로봇이 전방물체와 50cm이내 접근상태이면 True를 반환
    def _is_near(self) -> tuple[bool, tuple]:
        lidar_msg = self.laser_scan
        if lidar_msg:
            time = Time.from_msg(lidar_msg.header.stamp)
            # Run if only laser scan from simulation is updated
            if time.nanoseconds > self.nanoseconds:
                self.nanoseconds = time.nanoseconds

                ### Driving ###
                nearest_distance = 0.25  # 50cm

                distance_map = {}
                distance_map.update({0: min(lidar_msg.ranges[0:8])})  # 우측
                distance_map.update({1: min(lidar_msg.ranges[8:16])})
                distance_map.update({2: min(lidar_msg.ranges[16:24])})
                distance_map.update({3: min(lidar_msg.ranges[24:28])})

                distance_map.update({4: min(lidar_msg.ranges[28:37])})  # 중앙

                distance_map.update({5: min(lidar_msg.ranges[37:41])})  # 좌측
                distance_map.update({6: min(lidar_msg.ranges[41:49])})
                distance_map.update({7: min(lidar_msg.ranges[49:57])})
                distance_map.update({8: min(lidar_msg.ranges[57:65])})

                min_distance = distance_map.get(4)
                # map에서 value로 index를 찾는다.
                torq_map = {
                    0: -0.3,
                    1: -0.2,
                    2: -0.2,
                    3: -0.3,
                    4: -0.4,
                    5: -0.3,
                    6: -0.2,
                    7: -0.2,
                    8: -0.3,
                }
                min_index = next(
                    key for key, value in distance_map.items() if value <= min_distance
                )
                angle = -1.0 if min_index < 4 else -0.1 if min_index == 4 else 1.0
                torque = torq_map.get(min_index, 0.0)

                self.state_near = (torque, angle)

                self.get_logger().info(
                    f"distances:{min_distance}/기준:{nearest_distance}"
                )
                self.get_logger().info(f"[_is_near] index:{min_index}, T/A: {torque}/{angle}")
                return min_distance <= nearest_distance

        return False

    def send_command(self, request, response) -> None:
        self.get_logger().info(f"request: {request}")
        response.success = True
        if request.message == "reset":
            self.setup()
        elif request.message == "set_dest":
            x = int(request.data.x)
            y = int(request.data.y)
            self.set_destpos((x, y))
        else:
            response.success = False

        return response

    def _send_message(
        self, *, title: str = None, x: float = 0.0, theta: float = 0.0
    ) -> None:
        self.get_logger().info(f"#### {title} ### torque:{x}, angular:{theta} ####")
        self.twist_msg.linear = Vector3(x=x, y=0.0, z=0.0)
        self.twist_msg.angular = Vector3(x=0.0, y=0.0, z=theta)
        self.pub_jetauto_car.publish(self.twist_msg)

    def print_log(self, message) -> None:
        self.get_logger().info(message)

    def laser_scan(self, message: Message):
        self.laser_scan = message.data

    def update(self, message: Message):
        self.print_log(f"data_type: {message.data_type}, data: {message.data}")

        if message.data_type == "command":
            self._send_message(
                title=message.data.get("name"),
                x=float(message.data["torque"]),
                theta=float(message.data["theta"]),
            )

        elif message.data_type == "notify":
            self.step_completed = True
            # self.print_log(f'data_type: {message.data_type}, data: act_complete')

        # elif message.data_type == 'log':
        #     self.print_log(message.data)


def main(args=None):
    rclpy.init(args=args)

    path_node = AStartSearchNode()
    lidar_node = LidarScanNode()
    grid_node = GridMap()

    executor = MultiThreadedExecutor()
    executor.add_node(grid_node)
    executor.add_node(path_node)
    executor.add_node(lidar_node)

    try:
        executor.spin()
    finally:
        executor.shutdown()
        path_node.destroy_node()
        rclpy.shutdown()
