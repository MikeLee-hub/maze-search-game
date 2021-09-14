###### Write Your Library Here ###########

from collections import deque

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
    start_point=maze.startPoint()

    path=[]

    ####################### Write Your Code Here ################################

    path_list = []  # 경로를 저장할 이차원 배열 생성. 이 배열을 이용해 closed 여부도 파악할 수 있다.
    for i in range(maze.rows):  # x: rows , y : cols
        path_list.append([])
        for j in range(maze.cols):
            path_list[i].append(None)
    path_list[start_point[0]][start_point[1]] = [list(start_point)]

    frontiers = deque([start_point])  # bfs의 frontier을 저장할 fifo queue

    while len(frontiers) > 0:
        x, y = frontiers.popleft()
        for new_x, new_y in maze.neighborPoints(x, y):
            if maze.isWall(new_x, new_y):  # 다음 탐색할 좌표가 벽인지 확인
                continue
            if path_list[new_x][new_y]:  # 다음 탐색할 좌표가 이미 탐색한 좌표인지 확인 ( closed된 곳인지 확인 )
                continue

            path_list[new_x][new_y] = path_list[x][y] + [[new_x, new_y]]
            if maze.isObjective(new_x, new_y):
                return path_list[new_x][new_y]
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

    start_point=maze.startPoint()

    end_point=maze.circlePoints()[0]

    path=[]

    ####################### Write Your Code Here ################################

















    return path

    ############################################################################


# -------------------- Stage 02: Four circles - A* Algorithm  ------------------------ #



def stage2_heuristic():
    pass


def astar_four_circles(maze):
    """
    [문제 03] 제시된 stage2의 맵 세가지를 A* Algorithm을 통해 최단 경로를 return하시오.(30점)
    (단 Heurstic Function은 위의 stage2_heuristic function을 직접 정의하여 사용해야 한다.)
    """

    end_points=maze.circlePoints()
    end_points.sort()

    path=[]

    ####################### Write Your Code Here ################################


















    return path

    ############################################################################



# -------------------- Stage 03: Many circles - A* Algorithm -------------------- #

def mst(objectives, edges):

    cost_sum=0
    ####################### Write Your Code Here ################################













    return cost_sum

    ############################################################################


def stage3_heuristic():
    pass


def astar_many_circles(maze):
    """
    [문제 04] 제시된 stage3의 맵 세가지를 A* Algorithm을 통해 최단 경로를 return하시오.(30점)
    (단 Heurstic Function은 위의 stage3_heuristic function을 직접 정의하여 사용해야 하고, minimum spanning tree
    알고리즘을 활용한 heuristic function이어야 한다.)
    """

    end_points= maze.circlePoints()
    end_points.sort()

    path=[]

    ####################### Write Your Code Here ################################





















    return path

    ############################################################################
