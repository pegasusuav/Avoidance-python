# Avoidance-SUAS2019
Avoidance system for virtual static cylindrical shape obstacle during AUTO mission.
The system use MAVLink protocol to send MAVmsg direct to the UAV via MAVProxy GCS.

This code was create for the SUAS2019 challenge propose.

#### File description
```
- main.py					The main script.
- avd_algorithm.py			The avoidance algorithm script.
- arm_disarm.py				Testing MAVlink communication by send arm/disarm command. 
- avoid.py				Script use for test the avoidance algorithm.
- load_module.py
- mavlink_reader.py			Mavlink message reading test script.
- rcv_data.py				Mavlink message recieving test script.
- readwrite_test.py			Read/Write a csv file test script.
- testfloat.py        
- wp_sent.py				Send guide waypoint test script.
- new_point.csv				Store the guided waypoints in this file.
- new_point_test.csv		        Store testing waypoints in this file.
- obstacle.csv				Store the obstacle coordinate in this file.
```
