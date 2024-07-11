from rclpy.node import Node
from auto_runner.lib.common import TypeVar, SearchEndException, print_log
from auto_runner.lib.parts import *
from auto_runner.lib.path_manage import PathManage

LoggableNode = TypeVar("LoggableNode", bound=MessageHandler)


class RobotController2(Observer):
    action_map = {
        DirType.LEFT: (1.0, 1.35),
        DirType.RIGHT: (1.0, -1.35),
        DirType.FORWARD: (0.6, 0.0),
    }

    def __init__(self, node: Node):
        super().__init__()
        # 맵을 가진다.
        self.node = node
        self.pathMnger = PathManage(algorithm="a-star", dest_pos=(0, 0))
        self.orient: Orient = Orient.X
        self.cur_pos = (0, 0)
        self.path = [(-1, -1)]
        self.dest_pos = None
        IMUData.subscribe(o=self)

    def prepare(self) -> bool:
        imu_data = self.get_msg(only_body=True)
        # 방향, 현재위치, 회전각도 등 취합
        self.cur_pos: tuple = self.pathMnger.transfer2_xy(imu_data[:2])
        self.angular_data: float = imu_data[2]
        print_log(f"<<prepare>> {self.cur_pos} : {self.angular_data}")

        if self._need_new_path(self.cur_pos, self.dest_pos, self.path):
            # 목표경로를 새로 정한다.
            try:
                self.path = self.pathMnger.search_new_path(self.cur_pos, self.orient)
            except SearchEndException:
                return False
            self.dest_pos = self.path[-1]
        else:
            # 기존 목표경로를 사용한다.
            cur_ix = self.path.index(self.cur_pos)
            self.path = self.path[cur_ix:]
        return True

    def make_plan(self) -> EvHandle:
        """후진, 회전, 직진여부를 체크하고 해당 policy를 반환한다"""
        policy: EvHandle = Policy.search_action(self.orient, self.cur_pos, self.path)
        print_log(f"<<make_plan>> {policy=} :{policy.action=}")
        return policy

    def execute(self, next_plan: EvHandle, **kwargs):
        print_log(f"<<execute>> {next_plan}")
        _, self.orient = next_plan.action
        next_plan.apply(**kwargs)

    def _need_new_path(self, cur_pos:tuple, dest_pos:tuple, path) -> bool:
        """새로운 경로를 구해하는 조건"""
        is_first_try = not dest_pos
        is_destination_reached = cur_pos == dest_pos
        is_position_invalid = cur_pos not in path
        return is_first_try or is_destination_reached or is_position_invalid