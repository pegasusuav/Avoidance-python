import csv
import math
from pymavlink import mavutil


# Definition of all module function

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
        print("Changing %s mode" % modename)
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
        if msg.get_type() == 'GPS_RAW_INT':
            print("\n\n*****Got message: %s*****" % msg.get_type())
            print("Message: %s" % msg)
            print("\nAs dictionary: %s" % msg.to_dict())
            # Armed = MAV_STATE_STANDBY (4), Disarmed = MAV_STATE_ACTIVE (3)
            print("\nLatitude: %s" % msg.lat)
            print("\nLongitude: %s" % msg.lon)
            ref_lat = msg.lat / 10000000
            ref_lon = msg.lon / 10000000
            return ref_lat, ref_lon
        break


########################
# Send waypoint module #
########################
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
        go_lat * 10000000,  # lat_int
        go_lon * 10000000,  # lon_int
        30.0,  # alt
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0  # unused param
    )


##########################
# Update obstacle module #
##########################
def update_obs(the_connection):
    while True:
        '''
        open obstacle file
        check for obstacle distance
        if obstacle distance <= 30
        check current location
        calc distance using the Spherical Law of Cosines
        return avd_trigger

        '''
        # Read the obstacle.csv file and store Lat,Lon,Radius in variables
        with open('obstacle.csv') as obstacle_read:
            reader = csv.DictReader(obstacle_read)
            for row in reader:
                # Uncomment if using line number pointer
                # if row['No.'] == 1:
                obs_lat = float(row['Lat'])
                obs_lon = float(row['Lon'])
                obs_rad = float(row['Radius'])
        cur_lat, cur_lon = gps_data(the_connection)  # Check current position
        # Check distance between current position and obstacle (minus obstacle radius to make obstacle sheild)
        distance = ((math.acos(math.sin(cur_lat * math.pi / 180.0) * math.sin(obs_lat * math.pi / 180.0) + math.cos(
            cur_lat * math.pi / 180.0) * math.cos(obs_lat * math.pi / 180.0) * math.cos(
            obs_lon * math.pi / 180.0 - cur_lon * math.pi / 180.0)) * 6371000) - obs_rad)
        print(distance)
        # Return the avoidance algorithm trigger value if the distance below 30 meters
        if distance <= 30:
            return obs_lat, obs_lon, obs_rad
        break


# Testing field #
the_connection = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
the_connection.wait_heartbeat()
print("Got a Heartbeat from system (system %u component %u)" %
      (the_connection.target_system, the_connection.target_component))
gps_data(the_connection)
# update_obs(the_connection)
# flyto(-353614700,1491632130,the_connection)
