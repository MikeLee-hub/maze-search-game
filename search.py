###### Write Your Library Here ###########

import heapq
from collections import deque
from copy import deepcopy

#########################################


def search(maze, func):
    return {
        "bfs": bfs,
        "astar": astar,
        "astar_four_circles": astar_four_circles,
        "astar_many_circles": astar_many_circles
    }.get(func)(maze)


# -------------------- Stage 01: One circle - BFS Algorithm ------------------------ #

def bfs(maze):
    """
    [문제 01] 제시된 stage1의 맵 세가지를 BFS Algorithm을 통해 최단 경로를 return하시오.(20점)
    """
    start_point = maze.startPoint()
    ####################### Write Your Code Here ################################

    path_list = []                      # 경로를 저장할 이차원 배열 생성. 이 배열을 이용해 closed 여부도 파악할 수 있다.
    for i in range(maze.rows):          # x: rows , y : cols
        path_list.append([])
        for j in range(maze.cols):
            path_list[i].append(None)
    path_list[start_point[0]][start_point[1]] = [list(start_point)]

    frontiers = deque([start_point])    # bfs의 frontier을 저장할 fifo queue

    while len(frontiers) > 0:
        x, y = frontiers.popleft()

        if maze.isObjective(x, y):
            return path_list[x][y]
        for new_x, new_y in maze.neighborPoints(x, y):
            if maze.isWall(new_x, new_y):                   # 다음 탐색할 좌표가 벽인지 확인
                continue
            if path_list[new_x][new_y]:           # 다음 탐색할 좌표가 이미 탐색한 좌표인지 확인 ( closed된 곳인지 확인 )
                continue

            path_list[new_x][new_y] = path_list[x][y] + [[new_x, new_y]]
            frontiers.append([new_x, new_y])

    return []

    ############################################################################



class Node:
    def __init__(self,parent,location):
        self.parent=parent
        self.location=location #현재 노드

        self.obj=[]

        # F = G+H
        self.f=0
        self.g=0
        self.h=0

    def __eq__(self, other):
        return self.location==other.location and str(self.obj)==str(other.obj)

    def __le__(self, other):
        return self.g+self.h<=other.g+other.h

    def __lt__(self, other):
        return self.g+self.h<other.g+other.h

    def __gt__(self, other):
        return self.g+self.h>other.g+other.h

    def __ge__(self, other):
        return self.g+self.h>=other.g+other.h


# -------------------- Stage 01: One circle - A* Algorithm ------------------------ #

def manhatten_dist(p1,p2):
    return abs(p1[0]-p2[0])+abs(p1[1]-p2[1])

def astar(maze):

    """
    [문제 02] 제시된 stage1의 맵 세가지를 A* Algorithm을 통해 최단경로를 return하시오.(20점)
    (Heuristic Function은 위에서 정의한 manhatten_dist function을 사용할 것.)
    """

    start_point = maze.startPoint()

    end_point = maze.circlePoints()[0]

    ####################### Write Your Code Here ################################
    class frontier_node:
        def __init__(self, x, y, path):
            self.x = x
            self.y = y
            self.path = path
            self.f = manhatten_dist([x, y], end_point) + len(path)

        def __lt__(self, other):
            return self.f < other.f


    path_list = []  # 경로를 저장할 이차원 배열 생성. 이 배열을 이용해 closed 여부도 파악할 수 있다.
    for i in range(maze.rows):  # x: rows , y : cols
        path_list.append([])
        for j in range(maze.cols):
            path_list[i].append(None)
    frontiers = []
    heapq.heappush(frontiers, frontier_node(start_point[0], start_point[1], [list(start_point)]))  # bfs의 frontier을 저장할 fifo queue

    while len(frontiers) > 0:
        tmpnode = heapq.heappop(frontiers)
        x = tmpnode.x
        y = tmpnode.y
        if not path_list[x][y]:
            path_list[x][y] = tmpnode.path
        elif len(path_list[x][y]) > len(tmpnode.path):
            path_list[x][y] = tmpnode.path
        else:
            continue

        for new_x, new_y in maze.neighborPoints(x, y):
            if maze.isWall(new_x, new_y):  # 다음 탐색할 좌표가 벽인지 확인
                continue

            if maze.isObjective(new_x, new_y):
                return path_list[x][y] + [[new_x, new_y]]
            heapq.heappush(frontiers, frontier_node(new_x, new_y, path_list[x][y] + [[new_x, new_y]]))

    return []

    ############################################################################


# -------------------- Stage 02: Four circles - A* Algorithm  ------------------------ #



def stage2_heuristic(current_point, end_points):
    heuristic = 0
    for end_point in end_points:
        heuristic += manhatten_dist(current_point, end_point)
    return heuristic/len(end_points)

def astar_four_circles(maze):
    """
    [문제 03] 제시된 stage2의 맵 세가지를 A* Algorithm을 통해 최단 경로를 return하시오.(30점)
    (단 Heurstic Function은 위의 stage2_heuristic function을 직접 정의하여 사용해야 한다.)
    """

    end_points = maze.circlePoints()
    end_points.sort()

    ####################### Write Your Code Here ################################
    class frontier_node:            # frontier list의 node를 위한 class
        def __init__(self, g, current_point, end_points, before_states):
            self.current_point = current_point
            self.g = g
            self.h = stage2_heuristic(current_point, end_points)
            self.f = g + self.h
            self.end_points = end_points
            self.before_states = before_states

        def __lt__(self, other):    # priority queue 사용을 위해 객체 간 비교 속성 설정
            return self.f < other.f     # a* 알고리즘은 f(n)값에 따라 frontier 우선순위가 결정되므로 f값 비교

    start_point = maze.startPoint()

    visited = []
    for i in range(maze.rows):  # 이미 확인한 상태 체크. 상태는 current_point와 end_points로 구분
        visited.append([])
        for j in range(maze.cols):
            visited[i].append([])

    previous_states = dict()
    def trace_path(previous_state):     # previous_states정보를 바탕으로 path를 구성하는 함수
        path = []
        while previous_state != (start_point, tuple(end_points)):
            path.append(previous_state[0])
            previous_state = previous_states[previous_state[0], previous_state[1]]
        path.append(start_point)
        return path[::-1]

    frontiers = []          # frontier list (heapq를 이용하여 priority queue처럼 사용)
    heapq.heappush(frontiers, frontier_node(0, start_point, end_points, None))

    while len(frontiers) > 0:
        current_node = heapq.heappop(frontiers)

        current_x, current_y = current_node.current_point
        if current_node.end_points in visited[current_x][current_y]:    # 이미 체크한 상태이면 넘어가기
            continue
        visited[current_x][current_y].append(current_node.end_points)   # visited에 현재 상태 추가
        previous_states[current_node.current_point, tuple(current_node.end_points)] = current_node.before_states    # previous state 저장

        next_g = current_node.g + 1
        next_end_points = current_node.end_points
        if current_node.current_point in current_node.end_points:
            next_end_points = deepcopy(next_end_points)
            next_end_points.remove(current_node.current_point)
            if len(next_end_points) == 0:
                return trace_path(((current_x, current_y), tuple(current_node.end_points)))

        for next_x, next_y in maze.neighborPoints(current_x, current_y):
            if maze.isWall(next_x, next_y):
                continue
            heapq.heappush(frontiers, frontier_node(next_g,
                                                    (next_x, next_y),
                                                    deepcopy(next_end_points),
                                                    (current_node.current_point, tuple(current_node.end_points))))

    return []


    ############################################################################


# -------------------- Stage 03: Many circles - A* Algorithm -------------------- #
def mst(end_points, edge_cost_list, mst_cache):

    cost_sum = 0
    ####################### Write Your Code Here ################################
    if len(end_points) == 0:
        return 0
    if frozenset(end_points) in mst_cache:
        return mst_cache[frozenset(end_points)]
    original_end_points = deepcopy(end_points)

    checked_end_points = [end_points[0]]
    end_points = end_points[1:]
    while len(end_points) > 0:
        minimum_edge = end_points[0]
        mininum_edge_cost = edge_cost_list[checked_end_points[0], end_points[0]]
        for i in checked_end_points:
            for j in end_points:
                new_edge_cost = edge_cost_list[i, j]
                if new_edge_cost < mininum_edge_cost:
                    minimum_edge = j
                    mininum_edge_cost = new_edge_cost

        checked_end_points.append(minimum_edge)
        end_points.remove(minimum_edge)
        cost_sum += mininum_edge_cost
    mst_cache[frozenset(original_end_points)] = cost_sum
    return cost_sum

    ############################################################################

def make_tree(end_points, maze):

    def target_bfs(start, end, maze):
        path_list = []
        for i in range(maze.rows):
            path_list.append([])
            for j in range(maze.cols):
                path_list[i].append(None)
        path_list[start[0]][start[1]] = [list(start)]

        frontiers = deque([start])

        while len(frontiers) > 0:
            x, y = frontiers.popleft()

            if end[0] == x and end[1] == y:
                return path_list[x][y]
            for new_x, new_y in maze.neighborPoints(x, y):
                if maze.isWall(new_x, new_y):  # 다음 탐색할 좌표가 벽인지 확인
                    continue
                if path_list[new_x][new_y]:  # 다음 탐색할 좌표가 이미 탐색한 좌표인지 확인 ( closed된 곳인지 확인 )
                    continue

                path_list[new_x][new_y] = path_list[x][y] + [[new_x, new_y]]
                frontiers.append([new_x, new_y])
        return []

    edge_cost_list = dict()
    edge_path_list = dict()
    for i in range(len(end_points)-1):
        for j in range(i+1, len(end_points)):
            edge_path_list[end_points[i], end_points[j]] = target_bfs(end_points[i], end_points[j], maze)
            edge_path_list[end_points[j], end_points[i]] = edge_path_list[end_points[i], end_points[j]][::-1]
            edge_cost_list[end_points[i], end_points[j]] = len(edge_path_list[end_points[i], end_points[j]])
            edge_cost_list[end_points[j], end_points[i]] = edge_cost_list[end_points[i], end_points[j]]
    return edge_cost_list, edge_path_list


def astar_many_circles(maze):
    """
    [문제 04] 제시된 stage3의 맵 세가지를 A* Algorithm을 통해 최단 경로를 return하시오.(30점)
    (단 Heurstic Function은 위의 stage3_heuristic function을 직접 정의하여 사용해야 하고, minimum spanning tree
    알고리즘을 활용한 heuristic function이어야 한다.)
    """

    end_points = maze.circlePoints()
    end_points.sort()
    path = []

    ####################### Write Your Code Here ################################
    class frontier_node:  # frontier list의 node를 위한 class
        def __init__(self, current_point, end_points, g, h, prev):
            self.current_point = current_point
            self.g = g
            self.f = self.g + h
            self.end_points = end_points  # A deep copy
            self.prev = prev

        def __lt__(self, other):  # priority queue 사용을 위해 객체 간 비교 속성 설정
            return self.f < other.f  # a* 알고리즘은 f(n)값에 따라 frontier 우선순위가 결정되므로 f값 비교

    start_point = maze.startPoint()
    mst_cache = dict()

    edge_cost_list, edge_path_list = make_tree([start_point] + end_points, maze)
    frontiers = []
    heapq.heappush(frontiers, frontier_node(start_point, end_points, 0, mst(end_points, edge_cost_list, mst_cache), None))

    while len(frontiers) > 0:
        current_node = heapq.heappop(frontiers)

        if len(current_node.end_points) == 0:
            while current_node.prev:
                path += edge_path_list[current_node.current_point, current_node.prev.current_point][:-1]
                current_node = current_node.prev
            path.append(start_point)
            return path[::-1]

        for end_point in current_node.end_points:
            next_g = current_node.g + edge_cost_list[current_node.current_point, end_point]
            next_end_points = deepcopy(current_node.end_points)
            next_end_points.remove(end_point)
            heapq.heappush(frontiers, frontier_node(end_point, next_end_points, next_g, mst(next_end_points, edge_cost_list, mst_cache), current_node))

    return []

    ############################################################################
