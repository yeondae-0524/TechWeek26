import math
import heapq
from collections import deque


# =========================
# Common Interface
# =========================

UNKNOWN = -1
FREE = 0
OCCUPIED = 1


# =========================
# Utility
# =========================

NEIGHBORS = [
    (-1,0,1.0),
    (1,0,1.0),
    (0,-1,1.0),
    (0,1,1.0),

    (-1,-1,1.414),
    (-1,1,1.414),
    (1,-1,1.414),
    (1,1,1.414)
]


def inside(grid,r,c):

    return (
        0 <= r < len(grid)
        and
        0 <= c < len(grid[0])
    )



# =====================================================
# Frontier Exploration
# =====================================================

def find_frontiers(grid):

    frontiers=[]


    for r in range(len(grid)):

        for c in range(len(grid[0])):

            if grid[r][c] != FREE:
                continue


            for dr,dc,_ in NEIGHBORS:

                nr=r+dr
                nc=c+dc


                if inside(grid,nr,nc):

                    if grid[nr][nc] == UNKNOWN:

                        frontiers.append(
                            (r,c)
                        )

                        break


    return frontiers



def information_gain(
        grid,
        frontier,
        radius=4
):

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



def frontier_score(
        frontier,
        pose,
        grid,
        resolution=0.05
):

    r,c=frontier


    x,y,theta=pose


    # grid -> meter
    fx=c*resolution
    fy=r*resolution


    dx=fx-x
    dy=fy-y


    distance=math.sqrt(
        dx*dx+dy*dy
    )


    target_angle=math.atan2(
        dy,
        dx
    )


    angle_error=abs(
        target_angle-theta
    )


    angle_error=min(
        angle_error,
        2*math.pi-angle_error
    )


    gain=information_gain(
        grid,
        frontier
    )


    # 논문 방향 고려 frontier score
    score = (
        3.0*gain
        -
        2.0*distance
        -
        1.5*angle_error
    )


    return score



def select_frontier(
        grid,
        pose,
        resolution=0.05
):

    candidates=find_frontiers(grid)


    if not candidates:
        return None


    best=None
    best_score=-float("inf")


    for f in candidates:

        score=frontier_score(
            f,
            pose,
            grid,
            resolution
        )


        if score > best_score:

            best_score=score
            best=f


    return best




# =====================================================
# A*
# 8 Direction
# Costmap Support
# =====================================================


def heuristic(a,b):

    return math.sqrt(
        (a[0]-b[0])**2
        +
        (a[1]-b[1])**2
    )



def astar(
        grid,
        start,
        goal,
        costmap=None
):


    pq=[]


    heapq.heappush(
        pq,
        (0,start)
    )


    parent={
        start:None
    }


    g_cost={
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


        for dr,dc,move_cost in NEIGHBORS:


            nr=r+dr
            nc=c+dc


            if not inside(grid,nr,nc):
                continue



            if grid[nr][nc]==OCCUPIED:
                continue



            # costmap 적용
            extra=0


            if costmap:

                cost=costmap[nr][nc]


                if cost>=254:
                    continue


                extra=cost/100



            new_cost=(
                g_cost[current]
                +
                move_cost
                +
                extra
            )


            nxt=(nr,nc)


            if new_cost < g_cost.get(
                nxt,
                float("inf")
            ):


                g_cost[nxt]=new_cost


                f=(
                    new_cost
                    +
                    heuristic(
                        nxt,
                        goal
                    )
                )


                heapq.heappush(
                    pq,
                    (f,nxt)
                )


                parent[nxt]=current



    return []





# =====================================================
# Local Planner
# RPP-lite
# =====================================================


def local_planner(
        path,
        pose,
        local_costmap=None
):

    if not path:
        return 0,0


    x,y,theta=pose



    # waypoint 선택
    target=path[
        min(
            5,
            len(path)-1
        )
    ]


    row,col=target



    target_x=col*0.05
    target_y=row*0.05



    dx=target_x-x
    dy=target_y-y


    target_angle=math.atan2(
        dy,
        dx
    )


    error=target_angle-theta


    while error>math.pi:
        error-=2*math.pi


    while error<-math.pi:
        error+=2*math.pi



    # 방향 보정
    if abs(error)>0.35:

        return (
            0.0,
            1.5*error
        )



    speed=0.5



    # local costmap 위험 감속
    if local_costmap:


        front_cost=max(
            local_costmap
        )


        if front_cost>=200:

            speed=0.0



    return (
        speed,
        error
    )

def cluster_frontiers(frontiers):

    clusters = []
    visited = set()

    frontier_set = set(frontiers)

    for point in frontiers:

        if point in visited:
            continue

        cluster = []
        queue = deque([point])

        visited.add(point)

        while queue:

            current = queue.popleft()
            cluster.append(current)

            r,c = current

            for dr,dc,_ in NEIGHBORS:

                nxt = (r+dr,c+dc)

                if nxt in frontier_set and nxt not in visited:
                    visited.add(nxt)
                    queue.append(nxt)

        clusters.append(cluster)

    return clusters

def inflate_obstacles(grid, radius_cells=2):

    costmap = [
        [0 for _ in row]
        for row in grid
    ]

    for r in range(len(grid)):

        for c in range(len(grid[0])):

            if grid[r][c] == OCCUPIED:

                for dr in range(-radius_cells, radius_cells+1):

                    for dc in range(-radius_cells, radius_cells+1):

                        nr = r+dr
                        nc = c+dc


                        if not inside(grid,nr,nc):
                            continue


                        dist = abs(dr)+abs(dc)


                        if dist == 0:
                            cost = 254

                        else:
                            cost = max(
                                253 - dist*30,
                                1
                            )


                        costmap[nr][nc] = max(
                            costmap[nr][nc],
                            cost
                        )


    return costmap

