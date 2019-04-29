import time
from pymavlink import mavutil, mavwp

# Start a connection listening to a UDP port
the_connection = mavutil.mavlink_connection('udpin:localhost:14550')

# Wait for the first heartbeat 
#   This sets the system and component ID of remote system for the link
the_connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (the_connection.target_system, the_connection.target_component))

wp = mavwp.MAVWPLoader()

print(wp)

def set_waypoint_cmd_mavwp(lat,lon,alt):
    wp.add(mavutil.mavlink.MAVLink_mission_item_message(
        1,#sys id
        1,#component id
        0,#seq wp id
        0,#frame id
        16,#mav_cmd 
        0,#current, true/false
        0,#auto continue
        0,0,0,0,#params 1-4: hold time(s),acceptable radius(m), pass/orbit,yaw angle
        lat,lon,alt) #lat/lon/alt
    )

    print ("#of waypoints=",wp.count(),"\n")
    print ("waypoint is=",wp.wp(0),"\n")
    the_connection.waypoint_clear_all_send()
    the_connection.waypoint_count_send(wp.count())
    msg = the_connection.recv_match(type=['MISSION_REQUEST'],blocking=True)
    print (msg)
    seq=0
    the_connection.mav.send(wp.wp(msg.seq))



# set_waypoint_cmd_mavwp(-35.362767,149.162523,30)