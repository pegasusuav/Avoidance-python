from pymavlink import mavutil
import time


"""
Read and write parameters
"""


# Create the connection
the_connection = mavutil.mavlink_connection('udp:0.0.0.0:14550')
# Wait a heartbeat before sending commands
the_connection.wait_heartbeat()

# Request parameter
the_connection.mav.param_request_read_send(
    the_connection.target_system, the_connection.target_component,
    b'SURFACE_DEPTH',
    -1
)

# Print old parameter value
message = the_connection.recv_match(type='PARAM_VALUE', blocking=True).to_dict()
print('name: %s\tvalue: %d' % (message['param_id'].decode("utf-8"), message['param_value']))

time.sleep(1)

# Set parameter value
# Set a parameter value TEMPORARILY to RAM. It will be reset to default on system reboot.
# Send the ACTION MAV_ACTION_STORAGE_WRITE to PERMANENTLY write the RAM contents to EEPROM.
# The parameter variable type is described by MAV_PARAM_TYPE in http://mavlink.org/messages/common.
the_connection.mav.param_set_send(
    the_connection.target_system, the_connection.target_component,
    b'SURFACE_DEPTH',
    -12,
    mavutil.mavlink.MAV_PARAM_TYPE_REAL32
)

# Read ACK
# IMPORTANT: The receiving component should acknowledge the new parameter value by sending a param_value message to all communication partners.
# This will also ensure that multiple GCS all have an up-to-date list of all parameters.
# If the sending GCS did not receive a PARAM_VALUE message within its timeout time, it should re-send the PARAM_SET message.
message = the_connection.recv_match(type='PARAM_VALUE', blocking=True).to_dict()
print('name: %s\tvalue: %d' % (message['param_id'].decode("utf-8"), message['param_value']))

time.sleep(1)

# Request parameter value to confirm
the_connection.mav.param_request_read_send(
    the_connection.target_system, the_connection.target_component,
    b'SURFACE_DEPTH',
    -1
)

# Print new value in RAM
message = the_connection.recv_match(type='PARAM_VALUE', blocking=True).to_dict()
print('name: %s\tvalue: %d' % (message['param_id'].decode("utf-8"), message['param_value']))