from enum import Enum
from typing import TypeVar, Sequence
from auto_runner.lib.common import StateData, MessageHandler, State, Orient, DirType
import math
import threading
import time
import asyncio

state: State = State.ROTATE_STOP


class SensorData:
    tf_sensor: any
    lidar: any

    @classmethod
    def update_data(cls, tf_data: object, scan_data: any):
        cls.tf_sensor = tf_data
        cls.lidar = scan_data


class RotationManger:
    observers: list[object]
    start_angle: float

    def __init__(self, messagenser: MessageHandler):
        self.cmd = messagenser

    # 비동기로 callback
    @classmethod
    def add_observer(cls, o: object):
        cls.observers.append(o)
        
    @classmethod
    def notifyall(self, m: any):
        for o in self.observers:
            o.update(m)

    def start(self, callback: object, dir: Orient, orient: DirType = DirType.LEFT):
        self.dir = dir
        self.orient = orient
        self.target_angle = self.get_target_angle(dir, orient)
        self.add_observer(callback)

        global state
        while state != State.ROTATE_STOP:
            time.sleep(0.1)

        state = State.ROTATE_START

        threading(target=self.run).start()

    def run(self):
        while True:
            # 회전각 계산
            angle_diff = self.get_diff()

            # 회전완료 여부 체크
            if abs(angle_diff) < 0.2:                
                global state
                state = State.ROTATE_STOP
                self.notifyall(angle_diff)
                break

            sign_ = math.copysign(1, angle_diff)
            self.roate_angle(torque=0.15, sign=sign_)

            time.sleep(0.1)
    
    # 목표 각도 설정
    def get_target_angle(self, orient: Orient, dir: DirType) -> float:
        if (orient==Orient.X and dir==DirType.LEFT) or (orient==Orient._X and dir==DirType.RIGHT):
            return math.pi/2
        elif (orient==Orient.Y and dir==DirType.LEFT) or (orient==Orient._Y and dir==DirType.RIGHT):
            return math.pi
        elif (orient==Orient.X and dir==DirType.RIGHT) or (orient==Orient._Y and dir==DirType.LEFT):
            return 3*math.pi/2
        else:
            return 2*math.pi
        
    def roate_angle(self, torque: float, sign:int):
        self.cmd(torque=torque, angle=1.35*sign)

    # 반환값이 양수면 반시계방향으로 보정해야함
    def get_diff(self):
        cur_angle = SensorData.tf_sensor.angle % (2*math.pi)
        cur_angle = cur_angle if cur_angle > self.target_angle else (cur_angle + 2*math.pi)
        return self.target_angle - cur_angle

class MoveManager:
    def __init__(self, node: MessageHandler, state_data: StateData):
        self.node = node
        self.state_data = state_data


class ObstacleManager:
    def __init__(self, node: MessageHandler, state_data: StateData):
        self.node = node
        self.state_data = state_data

    # 회전과 이동이 구독
    def add_observer(self, o: object):
        self.observers.append(o)
