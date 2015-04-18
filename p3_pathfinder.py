from math import sqrt
from heapq import heappush, heappop, heapify


def dijkstras_shortest_path(src, dst, mesh):
    visited_boxes = []
    dist = {}
    dist[src] = 0
    prev = {}
    queue = []
    heappush(queue, (dist[src], src))

    while queue:
        _, min_box = heappop(queue)
        if min_box == dst:
            break
        for box in get_adjacent(mesh, min_box):
            visited_boxes.append(box)
            alt = dist[min_box] + box_dist(min_box, box)
            if (box not in dist) or alt < dist[box]:
                dist[box] = alt
                prev[box] = min_box
                if (_, box) in queue:
                    queue.remove(queue.index((_, box)))
                    queue.heapify()
                heappush(queue, (alt, box))

    # walk backwards down the path
    path = []
    if not (dst in prev):
        return []
    path.append(prev[dst])
    path.append(dst)
    while prev[path[0]] is not src:
        path.insert(0, prev[path[0]])
    path.insert(0, src)

    return path, visited_boxes


# mesh is an object
def get_adjacent(mesh, cell):
    return mesh['adj'][cell]


def box_center(box):
    box_x1, box_x2, box_y1, box_y2 = box
    return (box_x1 + box_x2) / 2, (box_y1 + box_y2) / 2


def box_dist(box1, box2):
    box1_x, box1_y = box_center(box1)
    box2_x, box2_y = box_center(box2)

    return sqrt((box1_x - box2_x) ** 2 + (box1_y - box2_y) ** 2)


def box_contains(box, point):
    box_x1, box_x2, box_y1, box_y2 = box
    point_x, point_y = point

    if (point_x < box_x1 and point_x < box_x2): return False
    if (point_x > box_x1 and point_x > box_x2): return False
    if (point_y < box_y1 and point_y < box_y2): return False
    if (point_y > box_y1 and point_y > box_y2): return False

    return True

# assumes adjacency
def find_closest_edge_point(start_point, start_box, target_box):
    assert box_contains(start_box, start_point)

    start_point_x, start_point_y = start_point
    target_box_x1, target_box_x2, target_box_y1, target_box_y2 = target_box

    #if in the bounds, remain the same
    target_point_x = start_point_x
    target_point_y = start_point_y

    # if outside the bounds, snap to corner
    if (start_point_x < target_box_x1 and start_point_x < target_box_x2):
        target_point_x = min(target_box_x1,target_box_x2)
    if (start_point_x > target_box_x1 and start_point_x > target_box_x2):
        target_point_x = max(target_box_x1,target_box_x2)
    if (start_point_y < target_box_y1 and start_point_y < target_box_y2):
        target_point_y = min(target_box_y1,target_box_y2)
    if (start_point_y > target_box_y1 and start_point_y > target_box_y2):
        target_point_y = max(target_box_y1,target_box_y2)

    return target_point_x, target_point_y

def find_path(src, dst, mesh):
    out_path = []
    visited_boxes = []

    for box in mesh['boxes']:
        if (box_contains(box, src)):
            src_box = box
        if (box_contains(box, dst)):
            dst_box = box

    assert src_box and dst_box

    out_path = []

    box_path, visited_boxes = dijkstras_shortest_path(src_box, dst_box, mesh)

    prev_x, prev_y = src
    prev_box = src_box
    for box in box_path:
        if box == src_box: continue
        x, y = find_closest_edge_point((prev_x,prev_y), prev_box, box)

        out_path.append(((prev_x,prev_y),(x,y)))

        prev_y = y
        prev_x = x
        prev_box = box
    
    x, y = dst
    out_path.append(((prev_x,prev_y),(x,y)))

    return out_path, visited_boxes  # list of points, list of boxes