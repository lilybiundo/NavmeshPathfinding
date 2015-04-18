from math import sqrt
from heapq import heappush, heappop, heapify

A_STAR = True
BIDIRECTIONAL = True

def shortest_path(start_point, end_point, src, dst, mesh):
    visited_boxes = []

    src_dist = {}
    src_dist[src] = 0
    src_prev = {}
    src_queue = []
    heappush(src_queue, (src_dist[src], src, start_point))

    dst_dist = {}
    dst_dist[dst] = 0
    dst_prev = {}
    dst_queue = []
    heappush(dst_queue, (dst_dist[dst], dst, end_point))

    src_dst_flop = True
    found = None

    if dst in get_adjacent(mesh, src):
        return [src, dst], [src, dst]

    while src_queue and dst_queue:
        if BIDIRECTIONAL and src_dst_flop:
            dist = dst_dist
            _, min_box, min_point = heappop(dst_queue)
            visited_boxes.append(min_box)
        else:
            dist = src_dist
            _, min_box, min_point = heappop(src_queue)
            visited_boxes.append(min_box)

        if not BIDIRECTIONAL:
            if min_box == dst:
                break

        for box in get_adjacent(mesh, min_box):
            # dist[min_box] is the distance of the path up to the min_box
            # step_dist is the step just made from min_box to box
            # box_dist(box, dst) is the A* estimate heuristic for remaining distance
            x, y = find_closest_edge_point(min_point, box)
            step_dist = euclid_dist(min_point, (x,y))
            alt = dist[min_box] + step_dist
            if A_STAR:
                if BIDIRECTIONAL and src_dst_flop:
                    alt += euclid_dist((x,y), start_point)
                else:
                    alt += euclid_dist((x,y), end_point)

            if (box not in dist) or alt < dist[box]:
                if BIDIRECTIONAL and src_dst_flop:
                    dst_dist[box] = alt
                    dst_prev[box] = min_box
                    if (_, box) in dst_queue:
                        dst_queue.remove(dst_queue.index((_, box)))
                        dst_queue.heapify()
                    heappush(dst_queue, (alt, box, (x,y)))
                else:
                    src_dist[box] = alt
                    src_prev[box] = min_box
                    if (_, box) in src_queue:
                        src_queue.remove(src_queue.index((_, box)))
                        src_queue.heapify()
                    heappush(src_queue, (alt, box, (x,y)))

            if BIDIRECTIONAL:
                # check for a meeting
                if src_dst_flop:
                    # remember this means dst is searching
                    if box in src_prev:
                        visited_boxes.append(box)
                        found = box
                        break
                else:
                    if box in dst_prev:
                        visited_boxes.append(box)
                        found = box
                        break

        if found != None:
            break
        src_dst_flop = not src_dst_flop

    # walk backwards down the path
    box_path = []
    if not BIDIRECTIONAL:
        # monodirectional search, if dst was found start from there
        if not (dst in src_prev):
            return [], visited_boxes
        box_path.append(dst)
    else:
        if found == None:
            return [], visited_boxes
        # bidirectional search, start at shared node
        box_path.append(found)

    while src_prev[box_path[0]] != src:
        box_path.insert(0, src_prev[box_path[0]])
    box_path.insert(0, src)

    if BIDIRECTIONAL:
        while dst_prev[box_path[-1]] != dst:
            box_path.append(dst_prev[box_path[-1]])
        box_path.append(dst)

    return box_path, visited_boxes

def euclid_dist(p1, p2):
    x1, y1 = p1
    x2, y2 = p2

    return sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

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
def find_closest_edge_point(start_point,target_box):
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

    src_box = None
    dst_box = None

    for box in mesh['boxes']:
        if (box_contains(box, src)):
            src_box = box
        if (box_contains(box, dst)):
            dst_box = box

    if not (src_box and dst_box):
        print "No path possible!"
        return [], visited_boxes

    if src_box == dst_box:
        return [(src,dst)], [src_box]

    out_path = []

    box_path, visited_boxes = shortest_path(src, dst, src_box, dst_box, mesh)

    if not box_path:
        print "No path possible!"
        return [], visited_boxes

    prev_x, prev_y = src
    prev_box = src_box
    for box in box_path:
        if box == src_box: continue
        x, y = find_closest_edge_point((prev_x,prev_y), box)

        out_path.append(((prev_x,prev_y),(x,y)))

        prev_y = y
        prev_x = x
        prev_box = box

    x, y = dst
    out_path.append(((prev_x,prev_y),(x,y)))

    return out_path, visited_boxes  # list of points, list of boxes