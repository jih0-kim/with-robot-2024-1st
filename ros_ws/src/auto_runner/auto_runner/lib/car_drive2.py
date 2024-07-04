from rclpy.node import Node
from auto_runner.lib.common import TypeVar, SearchEndException
from auto_runner.lib.parts import *
from auto_runner import mmr_sampling
from auto_runner.lib.path_manage import PathManage

LoggableNode = TypeVar("LoggableNode", bound=MessageHandler)
print_log = mmr_sampling.print_log


class RobotController2:
    action_map = {
        DirType.LEFT: (1.0, 1.35),
        DirType.RIGHT: (1.0, -1.35),
        DirType.FORWARD: (0.6, 0.0),
    }

    def __init__(self, node: Node):
        # 맵을 가진다.
        self.node = node
        self.pathMnger = PathManage(algorithm="a-star", dest_pos=(0, 0))
        self.orient: Orient = Orient.X
        self.cur_pos = (0, 0)
        self.path = [(-1, -1)]
        self.dest_pos = None
        IMUData.subscribe(o=self.get_msg)

    def get_msg(self, message: Message):
        self.imu_data = message.data

    def prepare(self):
        # 방향, 현재위치, 회전각도 등 취합
        self.cur_pos: tuple = self.pathMnger.transfer2_xy(self.imu_data[:2])
        self.angular_data: float = self.imu_data[2]
        print_log(f"<<prepare>> {self.cur_pos} : {self.angular_data}")

        # 목표경로를 정한다.
        if self._need_new_path():
            self.path = self.pathMnger.search_new_path(self.cur_pos, self.orient)
            self.dest_pos = self.path[-1] if self.path else []
        else:
            cur_ix = self.path.index(self.cur_pos)
            self.path = self.path[cur_ix:]

    def make_plan(self) -> Policy:
        """후진, 회전, 직진여부를 체크하고 해당 policy를 반환한다"""
        policy: EvHandle = Policy.search_action(self.orient, self.cur_pos, self.path)
        print_log(f"<<make_plan>> {policy=} :{policy.action=}")
        return policy

    def execute(self, next_plan: EvHandle, **kwargs):
        print_log(f"<<execute>> {next_plan}")
        _, self.orient = next_plan.action
        next_plan.apply(**kwargs)

    def _need_new_path(self) -> bool:
        """새로운 경로를 구해하는 조건"""
        is_destination_reached = self.cur_pos == self.dest_pos
        is_position_invalid = self.cur_pos not in self.path
        is_first_try = not self.dest_pos
        return is_first_try or is_destination_reached or is_position_invalid

    @property
    def check_arrived(self):
        cur_pos: tuple = self.pathMnger.transfer2_xy(self.imu_data[:2])
        return cur_pos == self.dest_pos
