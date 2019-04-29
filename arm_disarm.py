import time
from pymavlink import mavutil

# Start a connection listening to a UDP port
the_connection = mavutil.mavlink_connection('udpin:0.0.0.0:14550')

# Wait for the first heartbeat 
#   This sets the system and component ID of remote system for the link
the_connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (the_connection.target_system, the_connection.target_component))

# Once connected, use 'the_connection' to get and send messages
msg = None

# wait for the_connection connection
while msg is None:
        msg = the_connection.recv_msg()

print (msg)

# The values of these heartbeat fields is not really important here
# I just used the same numbers that QGC uses
# It is standard practice for any system communicating via mavlink emit the HEARTBEAT message at 1Hz! Your autopilot may not behave the way you want otherwise!
the_connection.mav.heartbeat_send(
6, # type
8, # autopilot
192, # base_mode
0, # custom_mode
4, # system_status
3  # mavlink_version
)

the_connection.mav.command_long_send(
1, # autopilot system id
1, # autopilot component id
400, # command id, ARM/DISARM
0, # confirmation
1, # arm!
0,0,0,0,0,0 # unused parameters for this command
)

time.sleep(5)

the_connection.mav.command_long_send(
1, # autopilot system id
1, # autopilot component id
400, # command id, ARM/DISARM
0, # confirmation
0, # disarm!
0,0,0,0,0,0 # unused parameters for this command
)