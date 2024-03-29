#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import TwistStamped, Twist, Vector3
from yolov8_msgs.srv import CmdMsg
from collections import deque
import math
import re


class AStartSearchNode(Node):

    ROT_TORQUE = 0.6  # 회전토크
    ROT_THETA = 1.35  # 회전각도크기
    FWD_TORQUE = 0.4  # 직진방향 토크

    # SLAM 맵 10X10
    SLAM_MAP = [
        [0, 0, 0, 1, 0, 1, 0, 0, 0, 0],
        [0, 0, 0, 1, 0, 1, 0, 0, 0, 0],
        [0, 0, 0, 1, 0, 1, 0, 0, 1, 0],
        [0, 0, 0, 0, 0, 1, 0, 0, 1, 0],
        [0, 1, 1, 0, 0, 1, 0, 0, 1, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 1, 1, 1, 1, 1, 1, 0],
        [0, 0, 0, 0, 0, 1, 0, 0, 0, 0],
        [0, 0, 1, 0, 0, 1, 0, 0, 0, 0],
        [0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
    ]
    # 진행방향 및 회전상태여부
    dir: str
    rotate_state: bool

    def __init__(self) -> None:
        super().__init__("path_search_astar")
        queue_size = 10
        # odom 위치정보 취득
        qos_profile = QoSProfile(depth=queue_size)
        self.create_subscription(
            TwistStamped,
            "/unity_tf",
            callback=self.unity_tf_sub_callback,
            qos_profile=qos_profile,
        )
        self.pub_jetauto_car = self.create_publisher(
            Twist, "jetauto_car/cmd_vel", queue_size
        )

        self.srv_jetauto_car = self.create_service(
            CmdMsg, "jetauto_car/cmd_msg", self.handle_reset
        )

        self.get_logger().info(f"AStartpath_searchNode started...")
        self.twist_msg = Twist()
        self.setup()

    def set_destpos(self, pos: tuple[int, int]) -> None:
        self.dest_pos = pos
        self.amend_h_count: int = 0
        self.is_arrived: bool = False
        self.get_logger().info(f"목적지 설정: {pos}")

    def setup(self) -> None:
        self._change_dir(dir="x", state=False)
        self.dest_pos = (2, 9)
        self.old_pos: tuple[int, int] = (-1, -1)
        self.prev_angular_z: float = -math.pi
        self.amend_h_count: int = 0
        self.is_arrived: bool = False
        self.get_logger().info("초기화 처리 완료")

    def handle_reset(self, request, response) -> None:
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

    def unity_tf_sub_callback(self, input_msg: TwistStamped) -> None:

        # msg로부터 위치정보를 추출
        current_pos = (input_msg.twist.linear.x, input_msg.twist.linear.y)

        # 좌표변환
        new_cor = self._get_cordinate(current_pos)
        if new_cor == self.dest_pos:
            # 정지 및 초기화처리
            self.stopat_dest(new_cor)
            return

            # 회전상태 체크
        if self.rotate_state:
            if self.is_rotating(input_msg.twist.angular):
                return

            self.amend_h_count = 0

        elif new_cor == self.old_pos:
            self._amend_h_pos(input_msg.twist.angular)
            return

        self.prev_angular_z = input_msg.twist.angular.z

        self.old_pos = new_cor

        # 네비게이션정보 취득.
        paths = self.find_path(new_cor, self.dest_pos)
        self.get_logger().info(f"A* PATH: {paths}")

        if len(paths) == 0:
            self.get_logger().info(f"길찾기 실패: 목표위치({self.dest_pos})")
            return

        # 다음번 위치에 대한 토크 및 회전각 취득.
        next_cor = paths[1]
        (x, theta), _next_dir = self._get_torq_theta(new_cor, next_cor, None)

        self._change_dir(dir=_next_dir, state=False if theta == 0 else True)
        if self.rotate_state:
            # 회전 전 감속
            self.send_breakmsg("_get_torq_theta", -self.FWD_TORQUE * 3 / 5)

        # 메시지 발행
        self.twist_msg.linear = Vector3(x=x, y=0.0, z=0.0)
        self.twist_msg.angular = Vector3(x=0.0, y=0.0, z=theta)
        self.pub_jetauto_car.publish(self.twist_msg)

        self.get_logger().info(f"현재:{new_cor}, 1차: {next_cor}")
        self.get_logger().info(f"토크: {x}, 회전: {theta}")
        self.get_logger().info(f"publish_msg: {self.twist_msg}")

    def stopat_dest(self, pos: tuple) -> None:
        if self.is_arrived:
            return
        self.get_logger().info(f"도착지[{pos}]에 도착했습니다.")
        self.twist_msg.angular = Vector3(x=0.0, y=0.0, z=0.0)
        self.twist_msg.linear = Vector3(x=0.0, y=0.0, z=0.0)
        self.pub_jetauto_car.publish(self.twist_msg)
        self.is_arrived = True

    # 회전여부를 파악한다.
    def is_rotating(self, angular: object) -> bool:

        TARGET_ANGLE = math.pi / 2

        def __angle_diff(angle1, angle2):
            if angle1 < angle2:
                angle1 += 2 * math.pi

            _diff = angle1 - angle2
            return (
                _diff
                if abs(_diff) < math.pi
                else (2 * math.pi - abs(_diff)) * _diff / abs(_diff)
            )

        rotation_angle = __angle_diff(angular.z, self.prev_angular_z)

        if TARGET_ANGLE * 0.8 < rotation_angle:
            break_torque = self.get_break_torque(rotation_angle, TARGET_ANGLE)
            self.send_breakmsg("is_rotating", break_torque)

            self.get_logger().info(
                f"angular_diff: {rotation_angle} / {rotation_angle < TARGET_ANGLE * 0.92}"
            )

        return rotation_angle < TARGET_ANGLE * 0.92

    # break신호
    def send_breakmsg(self, caller: str, torque: float) -> None:
        self.get_logger().info(f"[{caller}] break_torque: {torque}")
        self.twist_msg.angular = Vector3(x=0.0, y=0.0, z=0.0)
        self.twist_msg.linear = Vector3(x=0.0, y=0.0, z=torque)
        self.pub_jetauto_car.publish(self.twist_msg)

    def get_break_torque(
        self,
        angle: float,
        target_angle: float,
        max_torque: float = 0.07,
        min_torque: float = 0.02,
    ) -> float:
        """
        현재 회전각도에 따라 브레이크 토크의 크기를 결정한다.
        회전각도가 90도에 가까워질수록 토크 크기로 줄여나간다.
        """
        _torque = (
            (max_torque - min_torque)
            * math.log(angle + 1.0)
            / math.log(target_angle + 1.0)
        )
        return -_torque if _torque > min_torque else -min_torque

    # 몸체 평행상태 조정
    def _amend_h_pos(self, angular) -> tuple:

        amend_theta, message = self._get_deviation_radian(angular.z)
        # 간격을 두어 보정한다.
        # 잦은 조정에의한 좌우 흔들림 방지.
        if self.amend_h_count % 3 == 0 and abs(amend_theta) > 3 / 180 * math.pi:
            self.twist_msg.angular = Vector3(x=0.0, y=0.0, z=amend_theta * 3 / 5)
            self.twist_msg.linear = Vector3(x=0.6, y=0.0, z=0.0)
            self.pub_jetauto_car.publish(self.twist_msg)

            self.get_logger().info(
                f"[{message} 위치보정] 보정 각:{amend_theta} / 기준:{2 / 180 * math.pi}"
            )

        self.amend_h_count += 1

    # 각도를 입력받아, 진행방향과 벗어난 각도를 반환한다.
    def _get_deviation_radian(self, _z: float) -> tuple[float, str]:
        # 유니티에서 음의 값으로 제공.
        angular_z = -_z % (math.pi * 2)

        if self.dir == "x":
            amend_theta = angular_z - math.pi
            # z값이 이전치와 차이가 어느정도 나면 보정한다.
            message = "Z방향"

        elif self.dir == "-x":
            angular_z = angular_z if angular_z > math.pi else math.pi * 2 + angular_z
            amend_theta = angular_z - math.pi * 2
            message = "-Z방향"

        elif self.dir == "y":
            amend_theta = angular_z - math.pi / 2
            # z값이 이전치와 차이가 어느정도 나면 보정한다.
            message = "Y방향"

        elif self.dir == "-y":
            amend_theta = angular_z - math.pi * 3 / 2
            message = "-Y방향"
        else:
            amend_theta = 0.0
            message = "직진"

        return amend_theta, message

    """현재위치, 다음위치, 그 다음위치를 입력으로 받는다.
       앞방향 토크와 방향 각도를 반환한다.
    """

    def _get_torq_theta(
        self, cord0: tuple, cord1: tuple, cord2: tuple
    ) -> tuple[float, float]:
        # 다음위치와 그 다음위치를 확인해 회전방향을 계산한다.
        self.get_logger().info(f"_get_torq_theta_dir: {self.dir}")
        x0, y0 = cord0
        x1, y1 = cord2 if cord2 else cord1

        output = self.FWD_TORQUE, 0.0  # 직진
        _next_dir = self.dir
        if self.dir == "x":
            if y1 > y0:
                output = self.ROT_TORQUE, self.ROT_THETA  # 좌회전
                _next_dir = "y"
            elif y1 < y0:
                output = self.ROT_TORQUE, -self.ROT_THETA  # 우회전
                _next_dir = "-y"

        elif self.dir == "-x":
            if y1 > y0:
                output = self.ROT_TORQUE, -self.ROT_THETA  # 우회전
                _next_dir = "y"
            elif y1 < y0:
                output = self.ROT_TORQUE, self.ROT_THETA  # 좌회전
                _next_dir = "-y"

        elif self.dir == "y":
            if x1 > x0:
                output = self.ROT_TORQUE, -self.ROT_THETA  # 우회전
                _next_dir = "x"
            elif x1 < x0:
                output = self.ROT_TORQUE, self.ROT_THETA  # 좌회전
                _next_dir = "-x"

        elif self.dir == "-y":
            if x1 > x0:
                output = self.ROT_TORQUE, self.ROT_THETA  # 좌회전
                _next_dir = "x"
            elif x1 < x0:
                output = self.ROT_TORQUE, -self.ROT_THETA  # 우회전
                _next_dir = "-x"

        return output, _next_dir

    # 좌표취득
    def _get_cordinate(self, cord: tuple[float, float]) -> tuple[int, int]:
        # PC 맵 좌표를 SLAM 맵 좌표로 변환
        _x = math.floor(5.0 - cord[0])  # z
        _y = math.floor(5.0 - cord[1])  # y

        result = (_x if _x > 0 else 0, _y if _y > 0 else 0)
        return result

    def _change_dir(self, *, dir: str = None, state: bool = None) -> None:
        if dir:
            self.dir = dir
        if state is not None:
            self.rotate_state = state

    # a* search
    def find_path(
        self, start: tuple[int, int], goal: tuple[int, int]
    ) -> list[tuple[int, int]]:
        """
        A* 알고리즘을 사용하여 최단 경로를 찾습니다.
        :param grid: 2D 그리드 맵
        :param start: 시작 지점 (x, y)
        :param goal: 목표 지점 (x, y)
        :return: 최단 경로
        """
        self.get_logger().info(f"find_path: {start}, goal: {goal}")
        grid = self.SLAM_MAP
        start = (start[0], start[1], self.dir)

        frontier = deque()
        frontier.append((start, [start]))  # 큐에 시작 위치와 경로 추가
        visited = set()  # 방문한 노드 집합

        while frontier:
            curr_node, path = frontier.popleft()

            if curr_node[:2] == goal:
                # 초기입력된 방향값 제거
                path[0] = path[0][:2]
                return path

            if curr_node in visited:
                continue

            visited.add(curr_node)

            x, y, cur_d = curr_node
            for next_x, next_y, next_d in (
                (x + 1, y, "x"),
                (x - 1, y, "-x"),
                (x, y + 1, "y"),
                (x, y - 1, "-y"),
            ):
                if (
                    0 <= next_x < len(grid)
                    and 0 <= next_y < len(grid[0])
                    and grid[next_x][next_y] != 1
                ):
                    # 최초 다음위치에서 self.dir의 반대방향을 제외시킨다.
                    if self._check_if_backpath(cur_d, next_d):
                        continue

                    frontier.append(
                        ((next_x, next_y, next_d), path + [(next_x, next_y)])
                    )

            frontier = deque(
                sorted(
                    frontier,
                    key=lambda x: len(x[1]) + self._heuristic_distance(x[0][:2], goal),
                )
            )

        return []

    def _check_if_backpath(self, cur_dir: str, next_dir: str) -> bool:

        pattern = re.compile(r"^-(.)\1$|^(.)-\2$")
        # 후진 경로 배제
        return pattern.match(f"{cur_dir}{next_dir}") is not None

    def _heuristic_distance(self, a, b) -> int:
        """
        목표점까지의 추정거리를 추정한다
        """
        (x1, y1) = a
        (x2, y2) = b
        return abs(x1 - x2) + abs(y1 - y2)


def main(args=None):
    rclpy.init(args=args)

    path_node = AStartSearchNode()
    rclpy.spin(path_node)

    path_node.destroy_node()
    rclpy.shutdown()
