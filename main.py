#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import csv
import math
import avd_algorithm
import time
from pymavlink import mavutil

"""
Created on Thu Apr 25 19:30:42 2019

@author: Nuttawat Punpigul
"""

"""
Avoidance system for static cylindrical shape obstacle during AUTO mission
The system use MAVLink protocol to send MAVmsg direct to the UAV via MAVProxy GCS

The MAVLink protocol is hosted under the governance of the Dronecode Project.
See Wiki article (https://mavlink.io)
"""


# TODO: List
#  **** every coordinate points that writen in the csv file must have 7 decimals only ****
# - add mission complete check function
# - add AUTO mode check function before allow the script to continue


###################
# Haversine class #
###################
class Haversine:
    """
    use the haversine class to calculate the distance between
    two lon/lat coordinate pairs.
    output distance available in kilometers, meters, miles, and feet.
    example usage: Haversine([lon1,lat1],[lon2,lat2]).feet

    """

    def __init__(self, coord1, coord2):
        lon1, lat1 = coord1
        lon2, lat2 = coord2

        R = 6371000  # radius of Earth in meters
        phi_1 = math.radians(lat1)
        phi_2 = math.radians(lat2)

        delta_phi = math.radians(lat2 - lat1)
        delta_lambda = math.radians(lon2 - lon1)

        a = math.sin(delta_phi / 2.0) ** 2 + \
            math.cos(phi_1) * math.cos(phi_2) * \
            math.sin(delta_lambda / 2.0) ** 2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

        self.meters = R * c  # output distance in meters
        self.km = self.meters / 1000.0  # output distance in kilometers
        self.miles = self.meters * 0.000621371  # output distance in miles
        self.feet = self.miles * 5280  # output distance in feet


# Mode change function
def change_mode(modename, the_connection):
    # Choose a mode
    mode = modename
    # Receive the_connection for calling the function

    # Check if mode is available
    if mode not in the_connection.mode_mapping():
        print('Unknown mode : {}'.format(mode))
        print('Try:', list(the_connection.mode_mapping().keys()))
        exit(1)

    # Get mode ID
    mode_id = the_connection.mode_mapping()[mode]
    # Set new mode
    the_connection.mav.set_mode_send(
        the_connection.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id)

    # Check ACK
    # ack = False
    # while not ack:
    #     # Wait for ACK command
    #     ack_msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
    #     ack_msg = ack_msg.to_dict()

    #     # Check if command in the same in `set_mode`
    #     print("\nChanging %s mode" % modename)
    #     if ack_msg['command'] != mavutil.mavlink.MAVLINK_MSG_ID_SET_MODE:
    #         continue

    #     # Print the ACK result !
    #     result = mavutil.mavlink.enums['MAV_RESULT'][ack_msg['result']].description
    #     print("--> %s" % result)
    #     break
    return modename


# Read GPS data function
def gps_data(the_connection):
    while True:
        msg = the_connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        ref_lat = msg.lat
        ref_lon = msg.lon
        ref_alt = msg.relative_alt
        return ref_lat, ref_lon, ref_alt


# Guide waypoint function
# Read the new_point.csv file and get new waypoints coordinate for avoidance guiding
def get_guided_wp(select_row):
    with open('new_point.csv', 'r+') as newpoint_read:
        reader = csv.DictReader(newpoint_read)
        for row in reader:
            if int(row['No.']) == select_row:
                go_lat = row['Lat']
                go_lon = row['Lon']
                select_row -= 1
                return go_lat, go_lon, select_row


# Guided to waypoint function
def flyto(go_lat, go_lon, ref_alt, the_connection):
    # Loop for time_boot_ms parameter
    while True:
        msg = the_connection.recv_match()
        if not msg:
            continue
        if msg.get_type() == 'SYSTEM_TIME':
            break
    # Begin waypoint guiding
    the_connection.mav.set_position_target_global_int_send(
        msg.time_boot_ms,  # time_boot_ms
        the_connection.target_system,  # target_system
        the_connection.target_component,  # target_component
        6,  # coordinate_frame--> Use home alt as reference alt
        65528,  # type_mask
        int(go_lat),  # lat_int
        int(go_lon),  # lon_int
        ref_alt,  # alt
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)  # unused param
    # time.sleep(1)
    return


# Check guided waypoint reached function
def wp_reached(go_lat, go_lon, the_connection):
    while True:
        cur_lat, cur_lon, ref_alt = gps_data(the_connection)
        distance = Haversine((cur_lon / 10000000, cur_lat / 10000000),
                             (int(go_lon) / 10000000, int(go_lat) / 10000000)).meters
        print(distance)
        # /10000000 for convert int to float
        if distance <= 5:  # wp_reached offset (5)
            return


# TODO: check for non using param
# Update obstacle function
def update_obs(select_row):
    while True:
        # Read the obstacles.csv file and store Lat,Lon,Radius in variables
        with open('obstacles.csv') as f:
            count_obs = sum(1 for line in f) - 1
        with open('obstacles.csv', 'r+') as obstacle_read:
            reader = csv.DictReader(obstacle_read)
            while True:
                for row in reader:
                    if int(row['No.']) == select_row:  # Check the existence of obstacle
                        obs_lat = float(row['Lat'])
                        obs_lon = float(row['Lon'])
                        obstacle_read.close()
                        return obs_lat, obs_lon, count_obs
                    elif int(row['No.']) == 0:
                        print("No obstacle detect")
                        obstacle_read.close()
                        break
                break
        obstacle_read.close()


# Update obstacle distance function
def obstacle_dis(cur_lat, cur_lon, the_connection):
    while True:
        start_time = time.time()  # Start time record
        distance = []
        row = 1
        cur_lat, cur_lon, ref_alt = gps_data(the_connection)  # Check current position
        while True:
            obs_lat, obs_lon, count_obs = update_obs(row)  # Get obstacle data
            # Check distance between current position and obstacle shield
            distance.append(Haversine((cur_lon / 10000000, cur_lat / 10000000), (obs_lon, obs_lat)).meters)
            # /10000000 for convert int to float
            row += 1
            if row > count_obs:
                min_dist = min(distance)  # Get the nearest obstacle distance
                print('Closest obstacle distance = %f (--- %s seconds ---)' % (min_dist, (time.time() - start_time)))
                if min_dist <= 50.0:  # FIXME: <------- should be adjust by vehicle velocity and object rad (60.0 + obs_rad)
                    return min_dist
                else:
                    break


# Update current coordinate target function
def get_wp(the_connection):
    while True:
        msg = the_connection.recv_match()
        if not msg:
            continue
        if msg.get_type() == 'POSITION_TARGET_GLOBAL_INT':
            wp_lat = msg.lat_int
            wp_lon = msg.lon_int
            return wp_lat, wp_lon


# ------------------------------------------------------------------------------------------------------------------- #
def main():
    # Start a connection listening to a UDP port
    the_connection = mavutil.mavlink_connection('udpin:0.0.0.0:14550')

    # Wait for the first heartbeat
    #   This sets the system and component ID of remote system for the link
    the_connection.wait_heartbeat()
    print("Heartbeat from system (system %u component %u)" %
          (the_connection.target_system, the_connection.target_component))

    while True:
        ref_lat, ref_lon, ref_alt = gps_data(the_connection)  # Update current position
        obstacle_dis(ref_lat, ref_lon, the_connection)  # Check the nearest obstacle distance
        wp_lat, wp_lon = get_wp(the_connection)  # Update last waypoint
        # Run the algorithm
        print("\nObstacle in range")
        total_point = avd_algorithm.begin_avd(ref_lat, ref_lon, wp_lat, wp_lon, the_connection)
        # If obstacle distance is below 60 meters the guiding procedure will begin
        if total_point != 1:  # Prevent unnecessary mode changing
            print("Begin obstacle avoidance")
            print("\nChange to %s mode\n" % change_mode('GUIDED', the_connection))  # Change mode to GUIDED
            select_row = total_point  # Due to the algorithm write new waypoints from the end to start, we need to select
            # the last row as our first waypoint
            while select_row > 1:  # Delete the same destination waypoint
                if select_row == 0:
                    break
                print("Guiding...")
                # Fly to new waypoint in sequence
                go_lat, go_lon, select_row = get_guided_wp(select_row)  # Get new waypoint data
                ref_lat, ref_lon, ref_alt = gps_data(the_connection)
                flyto(go_lat, go_lon, ref_alt, the_connection)  # Guided to lat,lon point
                wp_reached(go_lat, go_lon, the_connection)  # Check waypoint reached
            print("Done\nChange to %s mode\n" % change_mode('AUTO', the_connection))  # Change mode to AUTO


# ------------------------------------------------------------------------------------------------------------------- #


if __name__ == '__main__':
    main()
