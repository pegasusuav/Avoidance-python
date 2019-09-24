#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import time
from pymavlink import mavutil

# Read GPS data function
def cur_wp(the_connection):
    while True:
        msg = the_connection.recv_match(type='MISSION_CURRENT', blocking=True)
        wp = msg.seq
        return wp


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

wpnum = int(input("No. of Airdrop sequence"))

# Start a connection listening to a UDP port
the_connection = mavutil.mavlink_connection('udpin:0.0.0.0:14554')

# Wait for the first heartbeat
#   This sets the system and component ID of remote system for the link
the_connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" %
        (the_connection.target_system, the_connection.target_component))

wp_num = cur_wp(the_connection)

if wp_num == (wpnum + 1):
    print("\nChange to %s mode\n" % change_mode('GUIDED', the_connection))  # Change mode to GUIDED