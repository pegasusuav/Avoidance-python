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
See Wiki article (https://mavlink.io/en/mavgen_python/)
"""


# TODO: List
#  **** every coordinate points that will be write in the csv file must have 7 decimals only ****
# - add mission complete check function
# - add AUTO mode check function before allow the script to continue
# / update obstacle status
# / store obstacle var
# / call get_gps function
# / check obstacle distance (the Haversine formula)
# / final distance = calc distance - obstacle_rad
# / if distance <= xx return avd_trigger
# / switch mode to guided (fix the 'can't avoid obstacle in time' problem)
# / call algorithm(obs_lat,obs_lon,obs_rad,ref_lat,ref_lon,the_connection)
# - Edit avd_algorithm.py to create new_point
# / Fly to new waypoint
# / check flyto reached (convert gps == flyto point)
# / sent next flyto
# / switch mode back to AUTO


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


######################
# Mode change module #
######################
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
    # the_connection.mav.command_long_send(
    #    the_connection.target_system, the_connection.target_component,
    #    mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
    #    0, mode_id, 0, 0, 0, 0, 0) or:
    # the_connection.set_mode(mode_id) or:
    the_connection.mav.set_mode_send(
        the_connection.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id)

    # Check ACK
    ack = False
    while not ack:
        # Wait for ACK command
        ack_msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
        ack_msg = ack_msg.to_dict()

        # Check if command in the same in `set_mode`
        print("\nChanging %s mode" % modename)
        if ack_msg['command'] != mavutil.mavlink.MAVLINK_MSG_ID_SET_MODE:
            continue

        # Print the ACK result !
        result = mavutil.mavlink.enums['MAV_RESULT'][ack_msg['result']].description
        print("--> %s" % result)
        break
    return modename


########################
# Read GPS data module #
########################
def gps_data(the_connection):
    while True:
        msg = the_connection.recv_match()
        if not msg:
            continue
        if msg.get_type() == 'GLOBAL_POSITION_INT':
            # print("\n\n*****Got message: %s*****" % msg.get_type())
            # print("Message: %s" % msg)
            # print("\nAs dictionary: %s" % msg.to_dict())
            # Armed = MAV_STATE_STANDBY (4), Disarmed = MAV_STATE_ACTIVE (3)
            # print("\nLatitude: %s" % msg.lat)
            # print("\nLongitude: %s" % msg.lon)
            ref_lat = msg.lat
            ref_lon = msg.lon
            return ref_lat, ref_lon


#########################
# Guide waypoint module #
#########################
# Read the new_point.csv file and get new waypoint coordinate
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
def flyto(go_lat, go_lon, the_connection):
    # Loop for time_boot_ms parameter
    while True:
        msg = the_connection.recv_match()
        if not msg:
            continue
        if msg.get_type() == 'SYSTEM_TIME':
            break
    # Begin position set
    the_connection.mav.set_position_target_global_int_send(
        msg.time_boot_ms,  # time_boot_ms
        the_connection.target_system,  # target_system
        the_connection.target_component,  # target_component
        6,  # coordinate_frame--> Use home alt as reference alt
        65528,  # type_mask
        int(go_lat),  # lat_int
        int(go_lon),  # lon_int
        30.0,  # alt
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)  # unused param
    # time.sleep(1)
    return


# Check guided waypoint reached function
def wp_reached(go_lat, go_lon, the_connection):
    while True:
        cur_lat, cur_lon = gps_data(the_connection)
        distance = Haversine((cur_lon / 10000000, cur_lat / 10000000),
                             (int(go_lon) / 10000000, int(go_lat) / 10000000)).meters
        # /10000000 for convert int to float
        # print(distance)
        if not distance <= 8:  # FIXME: <------- wp_reached offset
            continue
        return
        # if distance <= 0.25:
        #     return


# TODO: Can detect only one obstacle
##########################
# Update obstacle module #
##########################
def update_obs():
    while True:
        # Read the obstacle.csv file and store Lat,Lon,Radius in variables
        with open('obstacle.csv', 'r+') as obstacle_read:
            reader = csv.DictReader(obstacle_read)
            while True:
                for row in reader:
                    if row['No.'] != ' ':  # Check the existence of obstacle
                        obs_lat = float(row['Lat'])
                        obs_lon = float(row['Lon'])
                        obs_rad = float(row['Radius'])
                        obstacle_read.close()
                        return obs_lat, obs_lon, obs_rad
                    else:
                        print("No obstacle detect")
                        obstacle_read.close()
                        time.sleep(1)
                        break
                break
        obstacle_read.close()


###################################
# Update obstacle distance module #
###################################
def obstacle_dis(the_connection):
    while True:
        cur_lat, cur_lon = gps_data(the_connection)  # Check current position
        obs_lat, obs_lon, obs_rad = update_obs()  # Check that the obstacle
        # Check distance between current position and obstacle (minus obstacle radius to make obstacle shield)
        distance = Haversine((cur_lon / 10000000, cur_lat / 10000000), (obs_lon, obs_lat)).meters
        # /10000000 for convert int to float
        # if obs_lat != obs_lat1 or obs_lon != obs_lon1 or obs_rad != obs_rad1:
        #     return distance
        print('Obstacle distance = %f' % distance)
        # time.sleep(0.5)
        if distance <= 60.0 + obs_rad:  # FIXME: <------- should be adjust by vehicle velocity and object rad
            return distance


# TODO: review this function, maybe using MISSION_CURRENT {seq : x} with waypoint.csv file is better choice
###########################################
# Update current coordinate target module #
###########################################
def get_wp(the_connection):
    while True:
        msg = the_connection.recv_match()
        if not msg:
            continue
        if msg.get_type() == 'POSITION_TARGET_GLOBAL_INT':
            # print("\n\n*****Got message: %s*****" % msg.get_type())
            # print("Message: %s" % msg)
            # print("\nAs dictionary: %s" % msg.to_dict())
            # Armed = MAV_STATE_STANDBY (4), Disarmed = MAV_STATE_ACTIVE (3)
            # print("\nLatitude: %s" % msg.lat)
            # print("\nLongitude: %s" % msg.lon)
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
        obs_lat, obs_lon, obs_rad = update_obs()  # Update obstacle status
        obstacle_dis(the_connection)  # Check obstacle distance
        ref_lat, ref_lon = gps_data(the_connection)  # Update last position
        wp_lat, wp_lon = get_wp(the_connection)  # Update last waypoint
        # Run the algorithm
        total_point = avd_algorithm.begin_avd(ref_lat, ref_lon, wp_lat, wp_lon)
        # total_point = avd_algorithm.testing()
        # If obstacle distance is below 40 meters the guiding procedure will begin
        print("Obstacle in range\nBegin obstacle avoidance")
        print("----> Done change %s mode\n" % change_mode('GUIDED', the_connection))  # Change mode to GUIDED
        select_row = total_point
        while select_row > 1:  # Delete the same destination waypoint
            if select_row == 0:
                break
            print("Guiding...")
            go_lat, go_lon, select_row = get_guided_wp(select_row)  # Fly to new waypoint in sequence
            flyto(go_lat, go_lon, the_connection)  # Guided to lat,lon point
            wp_reached(go_lat, go_lon, the_connection)  # Check waypoint reached
        print("----> Done change %s mode" % change_mode('AUTO', the_connection))  # Change mode to AUTO


# ------------------------------------------------------------------------------------------------------------------- #


if __name__ == '__main__':
    main()
