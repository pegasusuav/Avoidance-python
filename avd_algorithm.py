#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import csv
import time
import matplotlib.pyplot as plt
from pymavlink import mavutil

"""
Created on Thu Apr 25 10:30:42 2019

@author: masonthammawichai, Marut M., Junior, Zulu
"""

"""
dijkstra grid based planning
author: Edsger W. Dijkstra
See Wikipedia article (https://en.wikipedia.org/wiki/Dijkstra%27s_algorithm)
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
    d = w * math.sqrt((n1.x - n2.x) ** 2 + (n1.y - n2.y) ** 2)
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

    xwidth = int(round(maxx - minx))
    ywidth = int(round(maxy - miny))

    # obstacle map generation
    obmap = [[False for i in range(ywidth)] for i in range(xwidth)]
    for ix in range(xwidth):
        x = ix + minx
        for iy in range(ywidth):
            y = iy + miny
            #  print(x, y)
            for iox, ioy in zip(ox, oy):
                d = math.sqrt((iox - x) ** 2 + (ioy - y) ** 2)
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


# Write wp function
def write_wp(num, lat, lon):
    head = [["No.", "Lat", "Lon"]]
    row = [[str(num), str(lat), str(lon)]]
    if int(num) == 1:
        with open('new_point.csv', 'w', newline='') as writeFile:
            writer = csv.writer(writeFile)
            writer.writerows(head)
            writer.writerows(row)
    else:
        with open('new_point.csv', 'a', newline='') as writeFile:
            writer = csv.writer(writeFile)
            writer.writerows(row)

    writeFile.close()


# Count total point quantity function
def count(filename):
    with open(filename) as f:
        return sum(1 for line in f)


# Select obstacle
def update_obs(select_row):
    with open('obstacles.csv', 'r+') as newpoint_read:
        reader = csv.DictReader(newpoint_read)
        for row in reader:
            if int(row['No.']) == select_row:
                obs_lat = row['Lat']
                obs_lon = row['Lon']
                obs_rad = row['Radius']
                return obs_lat, obs_lon, obs_rad

# Read GPS data function
def gps_data(the_connection):
    while True:
        msg = the_connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        ref_lat = msg.lat
        ref_lon = msg.lon
        return ref_lat, ref_lon


# All input
def begin_avd(ref_lat, ref_lon, wp_lat, wp_lon, the_connection):
    start_time = time.time()
    ref_lat,ref_lon = gps_data(the_connection)
    scale = 100000  # x/y scale

    # ref_lat,lon,scale database
    # RTAFA = 13.91818, 100.62812, 100000
    # MiniRC = 13.89443, 100.77626, 100000
    # NAS = 38.1405, -76.4354, 10000
    
    ref_fence_lon = 100.77626
    ref_fence_lat = 13.89443
    # start and goal position
    sx = ((ref_lon / 10000000) - ref_fence_lon) * scale  # [m] current positions
    sy = ((ref_lat / 10000000) - ref_fence_lat) * scale  # [m]
    gx = ((wp_lon / 10000000) - ref_fence_lon) * scale  # [m] next waypoints
    gy = ((wp_lat / 10000000) - ref_fence_lat) * scale  # [m]
    grid_size = 9  # [m]
    robot_radius = 9.0  # [m]
    # about obstacle
    ox, oy = [], []
    cx, cy = [], []
    r = []
    steps = 30  # [degrees]
    n = 0
    obs_num = int(count("obstacles.csv"))
    Total_Obs = obs_num - 1  # Count number of obstacle
    obs = 1
    while obs <= Total_Obs:
        obs_lat, obs_lon, obs_rad = update_obs(obs)  # get obstacle list
        r.append(float(obs_rad) / 30.0)  # [m] obstacle radius
        cx.append((float(obs_lon) - ref_fence_lon) * scale)
        cy.append((float(obs_lat) - ref_fence_lat) * scale)
        ox.append(cx[n])
        oy.append(cy[n])
        for i in range(0, 360, steps):
            ox.append(cx[n] + (r[n] * math.cos(i)))
            oy.append(cy[n] + (r[n] * math.sin(i)))
        n += 1
        obs += 1
    for i in range(418):
        ox.append(i)
        oy.append(417.0)
    for i in range(418):
        ox.append(0.0)
        oy.append(i)
    if show_animation:  # pragma: no cover
        plt.plot(ox, oy, ".k")
        plt.plot(sx, sy, "xr")
        plt.plot(gx, gy, "xb")
        plt.grid(True)
        plt.axis("equal")

    rx, ry = dijkstra_planning(sx, sy, gx, gy, ox, oy, grid_size, robot_radius)
    
    # TODO: Realtime plotting
    # Remove the comment in the rows that have "plt" in it to observe the plot graph
    # ---------------------------------------------------------------------------------------
    if show_animation:  # pragma: no cover

        plt.cla()
        plt.plot(rx, ry, "-r")
        prxb = int(rx[0])
        pryb = int(ry[0])
        if prxb == wp_lon:
            if pryb == wp_lat:
                return
        dx = 0
        dy = 0
        row_num = 1
        for prx, pry in zip(rx, ry):
            if (prxb - prx) != dx or (pryb - pry) != dy:
                plt.text(prxb, pryb, '({},{})'.format(prxb, pryb))
                guided_lat = int((pryb + (ref_fence_lat * 100000)) * 100)
                guided_lon = int((prxb + (ref_fence_lon * 100000)) * 100)
                write_wp(row_num, guided_lat, guided_lon)  # Write CSV file
                print((pryb + (ref_fence_lat * 100000)) / 100000, (prxb + (ref_fence_lon * 100000)) / 100000)
                row_num += 1
            dx = prxb - prx
            dy = pryb - pry
            prxb = prx
            pryb = pry
        plt.plot(ox, oy, ".k")
        plt.plot(sx, sy, "xr")
        plt.plot(gx, gy, "xb")
        plt.draw()
        plt.pause(0.05)
        plt.show(block=False)
        total_point = count("new_point.csv") - 1
        print("--- %s seconds ---" % (time.time() - start_time))
        return total_point
# --------------------------------------------------------------------------------------------


# # Testing zone
# if __name__ == '__main__':
#     ref_lat = 138953063
#     ref_lon = 1007781780
#     wp_lat = 138958895
#     wp_lon = 1007778990
#     total = begin_avd(ref_lat, ref_lon, wp_lat, wp_lon)
#     # begin_avd(ref_lat, ref_lon, wp_lat, wp_lon)
#     print(total)
