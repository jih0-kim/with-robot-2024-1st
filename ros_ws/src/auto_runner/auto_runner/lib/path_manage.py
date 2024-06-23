import math, re
from collections import deque
from auto_runner.lib.common import *
from auto_runner.lib.parts import MapData
from auto_runner import mmr_sampling
import time

LoggableNode = TypeVar("LoggableNode", bound=MessageHandler)
print_log = mmr_sampling.print_log

class PathManage:
    _grid_map = None
    def __init__(self, algorithm: str = "astar", dest_pos:tuple=(0,0)):
        self.path = []
        self.dest_pos = dest_pos
        self.updated = False        
        if algorithm == "astar":
            self.pathfinder = self._astar_method
        MapData.subscribe(o=self.update)

    @property
    def grid_map(self):
        if self.updated:
            self.updated = False
            return self._grid_map

    def update(self, message: Message):
        if message.data_type == "map" and self.updated == False:
            self._grid_map = message.data
            self.updated = True

    @property
    def all_path(self) -> list:
        return self.path

    def search_new_path(self, cur_pos: tuple, cur_orient: Orient) -> tuple:
        mmr_sampling.print_log(f"<<check_and_dest>> pos:{cur_pos}, dir:{cur_orient}")

        path = []
        count = 3
        exclude = self.path if len(self.path) > 0 else [cur_pos]
        while len(path) == 0 and count > 0:
            while not self.grid_map:
                time.sleep(0.1)
                
            dest_pos = mmr_sampling.find_farthest_coordinate(
                self.grid_map, cur_pos, exclude
            )
            if not dest_pos:
                mmr_sampling.print_log(f"mmr_sampling failed: map:{self.grid_map}")
                break

            path = self.pathfinder(cur_pos, dest_pos, cur_orient)
            if len(path) == 0:
                exclude.append(dest_pos)
                count -= 1

            mmr_sampling.print_log(f"목표위치: {dest_pos}, A* PATH: {path}")

        if len(path) == 0:
            return []

        self.dest_pos = dest_pos
        self.path = path
        return path

    # 위치값을 맵좌표로 변환
    def transfer2_xy(self, pose: tuple[float, float]) -> tuple[int, int]:
        # PC 맵 좌표를 SLAM 맵 좌표로 변환
        _x = math.floor(5.0 + pose[0])  # x
        _y = math.floor(5.0 + pose[1])  # y

        result = (_x if _x > 0 else 0, _y if _y > 0 else 0)
        print_log(f"<<_transfer2_xy>> {pose} => {result}")
        return result

    # a* method
    def _astar_method(
        self,
        start: tuple[int, int],
        goal: tuple[int, int],
        init_orient: Orient = Orient.X,
    ) -> list[list[int,int]]:
        """
        A* 알고리즘을 사용하여 최단 경로를 찾습니다.
        :param grid: 2D 그리드 맵
        :param start: 시작 지점 (x, y)
        :param goal: 목표 지점 (x, y)
        :return: 최단 경로
        """
        grid_map = self.grid_map

        if not grid_map:
            return []

        # 목표지점까지 추정거리
        def _heuristic_distance(self, start_pos, end_pos) -> int:
            """
            목표점까지의 추정거리를 추정한다
            """
            (x1, y1) = start_pos
            (x2, y2) = end_pos
            return abs(x1 - x2) + abs(y1 - y2)

        # 방향 제한조건
        def _check_if_backpath(self, cur_dir: Orient, next_dir: Orient) -> bool:
            # -xx or x-x 패턴
            pattern = re.compile(r"^-(.)\1$|^(.)-\2$")
            # 후진 경로 배제
            return pattern.match(f"{cur_dir.value}{next_dir.value}") is not None

        print_log(
            f"cur_pos: {start}, goal: {goal}, map:{len(self.grid_map.data)}"
        )
        
        frontier = deque()
        visited = set()  # 방문한 노드 집합

        start = (start[0], start[1], init_orient)
        frontier.append((start, [start]))  # 큐에 시작 위치와 경로 추가

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
                (x + 1, y, Orient.X),
                (x - 1, y, Orient._X),
                (x, y + 1, Orient.Y),
                (x, y - 1, Orient._Y),
            ):
                if (
                    0 <= next_x < len(grid_map[1])
                    and 0 <= next_y < len(grid_map[0])
                    and grid_map[next_x][next_y] != 1
                    # 최초 다음위치에서 self.dir의 반대방향을 제외시킨다.
                    and not _check_if_backpath(cur_d, next_d)
                ):
                    if _check_if_backpath(cur_d, next_d):
                        continue

                    frontier.append(
                        ((next_x, next_y, next_d), path + [(next_x, next_y)])
                    )

            frontier = deque(
                sorted(
                    frontier,
                    key=lambda x: len(x[1]) + _heuristic_distance(x[0][:2], goal),
                )
            )

        return []

    # 경로이탈 여부
    def _check_pose_error(self, cur_pos):
        if any(x for x in self.paths if x == cur_pos):
            return False

        return True