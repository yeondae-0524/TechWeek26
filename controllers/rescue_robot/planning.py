"""
Planning module
- Frontier exploration
- A* global planning
- Costmap inflation
- Local velocity helper

Interface:
grid:
    UNKNOWN=-1
    FREE=0
    OCCUPIED=1

pose:
    (x,y,theta)

path:
    [(row,col), ...]
"""

import heapq
import math
from collections import deque

import config
from interfaces import FREE, OCCUPIED, UNKNOWN


# =========================
# Neighbor
# =========================

NEIGHBORS_4 = (
    (-1,0,1.0),
    (1,0,1.0),
    (0,-1,1.0),
    (0,1,1.0)
)

NEIGHBORS = NEIGHBORS_4 + (
    (-1,-1,math.sqrt(2)),
    (-1,1,math.sqrt(2)),
    (1,-1,math.sqrt(2)),
    (1,1,math.sqrt(2))
)


def inside(grid,r,c):
    return (
        0 <= r < len(grid)
        and 0 <= c < len(grid[0])
    )


# =========================
# Frontier
# =========================

def find_frontiers(grid):
    """
    FREE와 UNKNOWN 경계 탐색
    """

    frontiers=[]

    for r,row in enumerate(grid):

        for c,value in enumerate(row):

            if value != FREE:
                continue


            for dr,dc,_ in NEIGHBORS:

                nr=r+dr
                nc=c+dc

                if inside(grid,nr,nc):
                    if grid[nr][nc]==UNKNOWN:
                        frontiers.append((r,c))
                        break


    return frontiers



def cluster_frontiers(frontiers,min_size=1):

    remain=set(frontiers)

    clusters=[]


    while remain:

        start=remain.pop()

        q=deque([start])

        cluster=[start]


        while q:

            r,c=q.popleft()


            for dr,dc,_ in NEIGHBORS:

                nxt=(r+dr,c+dc)


                if nxt in remain:

                    remain.remove(nxt)
                    q.append(nxt)
                    cluster.append(nxt)



        if len(cluster)>=min_size:
            clusters.append(cluster)



    return clusters



def information_gain(grid,frontier,radius=4):

    r,c=frontier

    gain=0


    for dr in range(-radius,radius+1):

        for dc in range(-radius,radius+1):

            nr=r+dr
            nc=c+dc

            if inside(grid,nr,nc):

                if grid[nr][nc]==UNKNOWN:
                    gain+=1


    return gain



def frontier_score(frontier,pose,grid,resolution,origin):

    r,c=frontier


    x = origin[0]+(c+0.5)*resolution
    y = origin[1]+(r+0.5)*resolution


    rx,ry,theta=pose


    dx=x-rx
    dy=y-ry


    distance=math.hypot(dx,dy)


    target_angle=math.atan2(dy,dx)


    angle_error=abs(
        math.atan2(
            math.sin(target_angle-theta),
            math.cos(target_angle-theta)
        )
    )


    gain=information_gain(grid,frontier)



    # 방향 고려 코스트 계산
    return (
        3.0*gain
        -2.0*distance
        -1.5*angle_error
    )



def select_frontier(grid,pose,resolution=None,origin=(0,0)):


    if resolution is None:
        resolution=config.GRID_RESOLUTION


    frontiers=find_frontiers(grid)


    if not frontiers:
        return None


    return max(
        frontiers,
        key=lambda f:
        frontier_score(
            f,
            pose,
            grid,
            resolution,
            origin
        )
    )



# =========================
# Costmap
# =========================


def inflate_obstacles(grid,radius_cells=2):

    """
    0~254 costmap 생성

    254 : 장애물
    100~253 : 위험지역
    0 : 안전
    """

    h=len(grid)
    w=len(grid[0])


    costmap=[
        [0]*w
        for _ in range(h)
    ]


    for r in range(h):

        for c in range(w):

            if grid[r][c]!=OCCUPIED:
                continue


            for dr in range(-radius_cells,radius_cells+1):

                for dc in range(-radius_cells,radius_cells+1):

                    nr=r+dr
                    nc=c+dc


                    if not inside(grid,nr,nc):
                        continue


                    dist=math.sqrt(
                        dr*dr+dc*dc
                    )


                    if dist==0:
                        cost=254

                    else:
                        cost=max(
                            1,
                            int(
                                254 -
                                dist/radius_cells*200
                            )
                        )


                    costmap[nr][nc]=max(
                        costmap[nr][nc],
                        cost
                    )


    return costmap



# =========================
# A*
# =========================


def heuristic(a,b):

    return math.hypot(
        a[0]-b[0],
        a[1]-b[1]
    )



def astar(
    grid,
    start,
    goal,
    costmap=None,
    allow_unknown=True,
    connectivity=8
):


    if not inside(grid,*start):
        return []


    if not inside(grid,*goal):
        return []



    neighbors = (
        NEIGHBORS_4
        if connectivity==4
        else NEIGHBORS
    )


    pq=[]

    heapq.heappush(
        pq,
        (0,start)
    )


    parent={
        start:None
    }


    cost={
        start:0
    }


    while pq:


        _,current=heapq.heappop(pq)


        if current==goal:

            path=[]

            while current:

                path.append(current)
                current=parent[current]


            return path[::-1]



        r,c=current


        for dr,dc,move in neighbors:


            nr=r+dr
            nc=c+dc


            nxt=(nr,nc)


            if not inside(grid,nr,nc):
                continue


            cell=grid[nr][nc]


            if cell==OCCUPIED:
                continue


            if cell==UNKNOWN and not allow_unknown:
                continue


            if dr and dc:

                if grid[r][nc]==OCCUPIED:
                    continue

                if grid[nr][c]==OCCUPIED:
                    continue



            extra=0


            if costmap:

                extra=costmap[nr][nc]/100


                if costmap[nr][nc]>=254:
                    continue



            new_cost=cost[current]+move+extra


            if nxt not in cost or new_cost<cost[nxt]:

                cost[nxt]=new_cost

                parent[nxt]=current


                heapq.heappush(
                    pq,
                    (
                        new_cost+heuristic(nxt,goal),
                        nxt
                    )
                )


    return []



# =========================
# Local helper
# =========================


def local_planner(path,pose):

    if not path:
        return 0.0,0.0


    r,c=path[-1]


    resolution=config.GRID_RESOLUTION


    gx=c*resolution
    gy=r*resolution


    x,y,theta=pose


    angle=math.atan2(
        gy-y,
        gx-x
    )


    error=math.atan2(
        math.sin(angle-theta),
        math.cos(angle-theta)
    )


    w=max(
        -config.MAX_ANGULAR_SPEED,
        min(
            config.MAX_ANGULAR_SPEED,
            error*2
        )
    )


    v=config.MAX_LINEAR_SPEED


    if abs(error)>0.8:
        v=0


    return v,w

