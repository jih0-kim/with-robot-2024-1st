import math, re
from collections import deque
from auto_runner.lib.common import *
from auto_runner import mmr_sampling

LoggableNode = TypeVar("LoggableNode", bound=MessageHandler)

class PathManage(Observer):
    grid_map = None
    def __init__(self, algorithm: str = "a-star", dest_pos:tuple=(0,0)):
        super().__init__()
        from auto_runner.lib.parts import MapData
        self.path = []
        self.dest_pos = dest_pos
        self.visited = []
        if algorithm == "a-star":
            self.pathfinder = self._astar_method
        MapData.subscribe(o=self)

    def search_new_path(self, cur_pos: tuple, cur_orient: Orient) -> tuple:
        print_log(f"<<check_and_dest>> pos:{cur_pos}, dir:{cur_orient}")

        max_try_count = 10
        exclude = [cur_pos] #self.path if len(self.path) > 0 else [cur_pos]
        exclude.extend(self.visited)
        path = []

        while len(path) == 0 and max_try_count > 0:
            grid_map = self.get_msg(only_body=True)

            dest_pos = mmr_sampling.find_farthest_coordinate(
                grid_map, cur_pos, exclude
            )
            if not dest_pos:
                print_log(f"mmr_sampling failed: map:{grid_map}")
                print_log(f"mmr_sampling failed: exclude:{exclude}")
                # raise SearchEndException("Path Finding ends")
                max_try_count -= 1
                continue

            path = self.pathfinder(cur_pos, dest_pos, cur_orient)
            if len(path) == 0:
                max_try_count -= 1
                exclude.append(dest_pos)

            print_log(f"목표위치: {dest_pos}, A* PATH: {path}")

        if len(path) == 0:
            raise SearchEndException("Path Finding ends")
        
        self.dest_pos = dest_pos
        self.path = path
        self.visited.append(dest_pos)
        return self.path

    @classmethod
    def transfer2_point(self, pose: tuple[int, int], origin:Orient, dir_type:DirType) -> tuple[float, float]:
        # PC 맵 좌표를 SLAM 맵 좌표로 변환
        if origin in [Orient.X, Orient.Y]:
            offset_x = 0.2 if dir_type==DirType.FORWARD else 0.9  # _x
            offset_y = 0.2 if dir_type==DirType.FORWARD else 0.9 # _y
        else:
            offset_x = 0.2 if dir_type==DirType.BACKWARD else 0.9  # _x
            offset_y = 0.2 if dir_type==DirType.BACKWARD else 0.9 # _y
        
        x = pose[0]-5.0+offset_x
        y = pose[1]-5.0+offset_y
        return (x, y)

    # 위치값을 맵좌표로 변환
    @classmethod
    def transfer2_xy(self, pose: tuple[float, float], origin:Orient=None, dir_type:DirType=None) -> tuple[int, int]:
        offset_x=offset_y=0
        if origin in [Orient.X, Orient.Y]:
            offset_x = -0.2 if dir_type==DirType.FORWARD else 0.5  # _x
            offset_y = -0.2 if dir_type==DirType.FORWARD else 0.5 # _y
        elif origin in [Orient._X, Orient._Y]:
            offset_x = -0.2 if dir_type==DirType.BACKWARD else 0.5  # _x
            offset_y = -0.2 if dir_type==DirType.BACKWARD else 0.5 # _y

        # PC 맵 좌표를 SLAM 맵 좌표로 변환
        _x = math.floor(5.0 + pose[0] + offset_x)  # x
        _y = math.floor(5.0 + pose[1] + offset_y)  # y

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
        grid_map = self.get_msg().data
        print_log(
            f"cur_pos: {start}, goal: {goal}, map:{grid_map}"
        )
        
        # 목표지점까지 추정거리
        def _heuristic_distance(start_pos, end_pos) -> int:
            """
            목표점까지의 추정거리를 추정한다
            """
            (x1, y1) = start_pos
            (x2, y2) = end_pos
            return abs(x1 - x2) + abs(y1 - y2)

        # 방향 제한조건
        def _check_if_backpath(cur_dir: Orient, next_dir: Orient) -> bool:
            # -xx or x-x 패턴
            pattern = re.compile(r"^-(.)\1$|^(.)-\2$")
            # 후진 경로 배제
            return pattern.match(f"{cur_dir.value}{next_dir.value}") is not None

        # queue
        frontier = deque()
        visited = set()  # 방문한 노드 집합

        start = (start[0], start[1], init_orient)
        frontier.append((start, [start], 0))  # 큐에 시작 위치와 경로 추가

        while frontier:
            cur_node, path, cost = frontier.popleft()

            if cur_node[:2] == goal:
                # 초기입력된 방향값 제거
                path[0] = path[0][:2]
                return path

            if cur_node in visited:
                continue

            visited.add(cur_node)

            x, y, cur_d = cur_node
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
                    # and not _check_if_backpath(cur_d, next_d)
                ):
                    cost = 10 if _check_if_backpath(cur_d, next_d) else 0

                    frontier.append(
                        ((next_x, next_y, next_d), path + [(next_x, next_y)], cost)
                    )

            frontier = deque(
                sorted(
                    frontier,
                    key=lambda x: len(x[1]) + x[2] + _heuristic_distance(x[0][:2], goal),
                )
            )

        return []

    # 경로이탈 여부
    def _check_pose_error(self, cur_pos):
        if any(x for x in self.paths if x == cur_pos):
            return False

        return True