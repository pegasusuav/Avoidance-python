#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import matplotlib.pyplot as plt
import math
import csv
import numpy as np

"""
Created on Thu Apr 25 10:30:42 2019

@author: masonthammawichai, Marut M., Junior, Zulu
"""

"""
A* grid based planning
author: Atsushi Sakai(@Atsushi_twi)
        Nikos Kanargias (nkana@tee.gr)
See Wikipedia article (https://en.wikipedia.org/wiki/A*_search_algorithm)
"""

show_animation = True


class Node:

    def __init__(self, x, y, cost, pind):
        self.x = x
        self.y = y
        self.cost = cost
        self.pind = pind

    def __str__(self):
        return str(self.x) + "," + str(self.y) + "," + str(self.cost) + "," + str(self.pind)


def calc_final_path(ngoal, closedset, reso):
    # generate final course
    rx, ry = [ngoal.x * reso], [ngoal.y * reso]
    pind = ngoal.pind
    while pind != -1:
        n = closedset[pind]
        rx.append(n.x * reso)
        ry.append(n.y * reso)
        pind = n.pind

    return rx, ry


def a_star_planning(sx, sy, gx, gy, ox, oy, reso, rr):
    """
    gx: goal x position [m]
    gx: goal x position [m]
    ox: x position list of Obstacles [m]
    oy: y position list of Obstacles [m]
    reso: grid resolution [m]
    rr: robot radius[m]
    """

    nstart = Node(round(sx / reso), round(sy / reso), 0.0, -1)
    ngoal = Node(round(gx / reso), round(gy / reso), 0.0, -1)
    ox = [iox / reso for iox in ox]
    oy = [ioy / reso for ioy in oy]

    obmap, minx, miny, maxx, maxy, xw, yw = calc_obstacle_map(ox, oy, reso, rr)

    motion = get_motion_model()

    openset, closedset = dict(), dict()
    openset[calc_index(nstart, xw, minx, miny)] = nstart

    while 1:
        c_id = min(
            openset, key=lambda o: openset[o].cost + calc_heuristic(ngoal, openset[o]))
        current = openset[c_id]

        # show graph
        if show_animation:  # pragma: no cover
            plt.plot(current.x * reso, current.y * reso, "xc")
            if len(closedset.keys()) % 10 == 0:
                plt.pause(0.001)

        if current.x == ngoal.x and current.y == ngoal.y:
            print("Find goal")
            ngoal.pind = current.pind
            ngoal.cost = current.cost
            break

        # Remove the item from the open set
        del openset[c_id]
        # Add it to the closed set
        closedset[c_id] = current

        # expand search grid based on motion model
        for i, _ in enumerate(motion):
            node = Node(current.x + motion[i][0],
                        current.y + motion[i][1],
                        current.cost + motion[i][2], c_id)
            n_id = calc_index(node, xw, minx, miny)

            if n_id in closedset:
                continue

            if not verify_node(node, obmap, minx, miny, maxx, maxy):
                continue

            if n_id not in openset:
                openset[n_id] = node  # Discover a new node
            else:
                if openset[n_id].cost >= node.cost:
                    # This path is the best until now. record it!
                    openset[n_id] = node

    rx, ry = calc_final_path(ngoal, closedset, reso)

    return rx, ry


def dijkstra_planning(sx, sy, gx, gy, ox, oy, reso, rr):
    """
    gx: goal x position [m]
    gx: goal x position [m]
    ox: x position list of Obstacles [m]
    oy: y position list of Obstacles [m]
    reso: grid resolution [m]
    rr: robot radius[m]
    """

    nstart = Node(round(sx / reso), round(sy / reso), 0.0, -1)
    ngoal = Node(round(gx / reso), round(gy / reso), 0.0, -1)
    ox = [iox / reso for iox in ox]
    oy = [ioy / reso for ioy in oy]

    obmap, minx, miny, maxx, maxy, xw, yw = calc_obstacle_map(ox, oy, reso, rr)

    motion = get_motion_model()

    openset, closedset = dict(), dict()
    openset[calc_index(nstart, xw, minx, miny)] = nstart

    while 1:
        c_id = min(openset, key=lambda o: openset[o].cost)
        current = openset[c_id]
        #  print("current", current)

        # show graph
        '''
        if show_animation:
            plt.plot(current.x * reso, current.y * reso, "xc")
            if len(closedset.keys()) % 10 == 0:
                plt.pause(0.001)
        '''
        if current.x == ngoal.x and current.y == ngoal.y:
            print("Find goal")
            ngoal.pind = current.pind
            ngoal.cost = current.cost
            break

        # Remove the item from the open set
        del openset[c_id]
        # Add it to the closed set
        closedset[c_id] = current

        # expand search grid based on motion model
        for i, _ in enumerate(motion):
            node = Node(current.x + motion[i][0], current.y + motion[i][1],
                        current.cost + motion[i][2], c_id)
            n_id = calc_index(node, xw, minx, miny)

            if not verify_node(node, obmap, minx, miny, maxx, maxy):
                continue

            if n_id in closedset:
                continue
            # Otherwise if it is already in the open set
            if n_id in openset:
                if openset[n_id].cost > node.cost:
                    openset[n_id].cost = node.cost
                    openset[n_id].pind = c_id
            else:
                openset[n_id] = node

    rx, ry = calc_final_path(ngoal, closedset, reso)

    return rx, ry


def calc_heuristic(n1, n2):
    w = 1.0  # weight of heuristic
    d = w * math.sqrt((n1.x - n2.x)**2 + (n1.y - n2.y)**2)
    return d


def verify_node(node, obmap, minx, miny, maxx, maxy):

    if node.x < minx:
        return False
    elif node.y < miny:
        return False
    elif node.x >= maxx:
        return False
    elif node.y >= maxy:
        return False

    if obmap[node.x][node.y]:
        return False

    return True


def calc_obstacle_map(ox, oy, reso, vr):

    minx = round(min(ox))
    miny = round(min(oy))
    maxx = round(max(ox))
    maxy = round(max(oy))
    #  print("minx:", minx)
    #  print("miny:", miny)
    #  print("maxx:", maxx)
    #  print("maxy:", maxy)

    xwidth = int(round(maxx - minx))
    ywidth = int(round(maxy - miny))
    #  print("xwidth:", xwidth)
    #  print("ywidth:", ywidth)

    # obstacle map generation
    obmap = [[False for i in range(ywidth)] for i in range(xwidth)]  # TODO: Fix local var 'i' not in use
    for ix in range(xwidth):
        x = ix + minx
        for iy in range(ywidth):
            y = iy + miny
            #  print(x, y)
            for iox, ioy in zip(ox, oy):
                d = math.sqrt((iox - x)**2 + (ioy - y)**2)
                if d <= vr / reso:
                    obmap[ix][iy] = True
                    break

    return obmap, minx, miny, maxx, maxy, xwidth, ywidth


def calc_index(node, xwidth, xmin, ymin):
    return (node.y - ymin) * xwidth + (node.x - xmin)


def get_motion_model():
    # dx, dy, cost
    motion = [[1, 0, 1],
              [0, 1, 1],
              [-1, 0, 1],
              [0, -1, 1],
              [-1, -1, math.sqrt(2)],
              [-1, 1, math.sqrt(2)],
              [1, -1, math.sqrt(2)],
              [1, 1, math.sqrt(2)]]

    return motion


# TODO: Write wp function and make it count point number quantity
def write_wp(num,lat,lon):
    head = [["No.", "Lat", "Lon"]]
    row = [[str(num), str(lat), str(lon)]]
    if int(num) == 1:
        with open('new_point.csv', 'w', newline = '') as writeFile:
            writer = csv.writer(writeFile)
            writer.writerows(head)
            writer.writerows(row)
    else:
        with open('new_point.csv', 'a', newline = '') as writeFile:
            writer = csv.writer(writeFile)
            writer.writerows(row)

    writeFile.close()


# Count total point quantity function
def count():
    filename = "new_point.csv"
    with open(filename) as f:
        return sum(1 for line in f)


# Manual plot testing function
def testing():
    total_point = 2
    return total_point


# All input
def begin_avd(ref_lat, ref_lon, wp_lat, wp_lon, obs_lat, obs_lon, obs_rad):
    # start and goal position
    sx = ((ref_lon/10000000)+76.4354)*10000  # [m] current positions
    sy = ((ref_lat/10000000)-38.1405)*10000  # [m]
    gx = ((wp_lon/10000000)+76.4354)*10000  # [m] next waypoints
    gy = ((wp_lat/10000000)-38.1405)*10000   # [m]
    grid_size = 5  # [m]
    robot_size = 3.0  # [m]

    ox, oy = [], []
    r = obs_rad  # [m] obstacle radius
    steps = 30  # [degrees]
    (cx, cy) = [(obs_lon+76.4354)*10000, (obs_lat-38.1405)*10000]  # [m] center of obstacle
    ox.append(cx)
    oy.append(cy)
    for i in range(0, 360, steps):
        ox.append(cx+(r*math.cos(i)))
        oy.append(cy+(r*math.sin(i)))
    for i in range(143):
        ox.append(i)
        oy.append(142.0)
    for i in range(143):
        ox.append(0.0)
        oy.append(i)
    # if show_animation:  # pragma: no cover
    #     plt.plot(ox, oy, ".k")
    #     plt.plot(sx, sy, "xr")
    #     plt.plot(gx, gy, "xb")
    #     plt.grid(True)
    #     plt.axis("equal")

    # rx, ry = a_star_planning(sx, sy, gx, gy, ox, oy, grid_size, robot_size)
    rx, ry = dijkstra_planning(sx, sy, gx, gy, ox, oy, grid_size, robot_size)
# TODO: Remove plot process to make the code run faster,
#  find waypoint printing part
# ---------------------------------------------------------------------------------------
    if show_animation:  # pragma: no cover
        # plt.plot(rx, ry, "-r")
        prxb = int(rx[0])
        pryb = int(ry[0])
        if prxb == wp_lon:
            if pryb == wp_lat:
                return
        dx = 0
        dy = 0
        row_num = 1
        for prx, pry in zip(rx, ry):
            # print("P", prx,pry)
            # print("x", prxb-prx)
            # print("y", pryb-pry)
            if (prxb-prx) != dx or (pryb-pry) != dy:
                plt.text(prxb, pryb, '({},{})'.format(prxb, pryb))
                guided_lat = int((pryb+381405)*10)
                guided_lon = int((prxb-764354)*10)
                write_wp(row_num, guided_lat, guided_lon)  # Write CSV file
                print((pryb+381405)*10, (prxb-764354)*10)
                row_num += 1
            dx = prxb-prx
            dy = pryb-pry
            prxb = prx
            pryb = pry
        # plt.plot(ox, oy, ".k")
        # plt.plot(sx, sy, "xr")
        # plt.plot(gx, gy, "xb")
        # plt.show()
        total_point = count() - 1
        return total_point
# --------------------------------------------------------------------------------------------


if __name__ == '__main__':
    ref_lat = 381497980
    ref_lon = -764280940
    wp_lat = 381482960
    wp_lon = -764315630
    obs_lat = 38.146444
    obs_lon = -76.429821
    obs_rad = 35.0
    total = begin_avd(ref_lat, ref_lon, wp_lat, wp_lon, obs_lat, obs_lon, obs_rad)
    print(total)