import json
import csv
import os
import sys
from auvsi_suas.client import client
from auvsi_suas.proto import interop_api_pb2
from google.protobuf import json_format

"""
Created on Thu Jun 13 09:30:42 2019

@author: Nuttawat Punpigul
"""

"""
The interop imprementation script for SUAS2019 competition.
The script will get the mission from the interop servo then reformat the data to CSV and TXT files which ready to use with another script made by Pegasus team.

The MAVLink protocol is hosted under the governance of the Dronecode Project.
See Wiki article (https://mavlink.io)
Interoperability System for the AUVSI SUAS Competition.
See Github (https://github.com/auvsi-suas/interop#interop-integration)
"""

# The example of JSON data for script testing
# mission = '{"id": 1,"flyZones": [{"altitudeMax": 750.0,"altitudeMin": 0.0,"boundaryPoints": [{"latitude": 38.142544,"longitude": -76.434088},{"latitude": 38.141833,"longitude": -76.425263},{"latitude": 38.144678,"longitude": -76.427995}]}],"searchGridPoints": [{"latitude": 38.142544,"longitude": -76.434088}],"offAxisOdlcPos": {"latitude": 38.142544,"longitude": -76.434088},"waypoints": [{"latitude": 38.142666,"altitude": 50.0,"longitude": -76.436777},{"latitude": 38.142544,"altitude": 200.0,"longitude": -76.434088}],"airDropPos":{"latitude": 38.141833,"longitude": -76.425263},"emergentLastKnownPos": {"latitude": 38.145823,"longitude": -76.422396},"stationaryObstacles": [{"latitude": 38.14792,"radius": 150.0,"longitude": -76.427995,"height": 200.0},{"latitude": 38.145823,"radius": 50.0,"longitude": -76.422396,"height": 300.0}]}'


def write_bp(num, lat, lon):
  home_lat = 38.144843
  home_lon = -76.427984
  row = [str(lat), str(lon)]
  homerow = [home_lat, home_lon]
  if int(num) == 0:
    with open('1.geofence.fen', 'w') as writer:
      writer.write('\t'.join(map(str,homerow)))
      writer.write('\n')
      writer.write('\t'.join(map(str,row)))
      writer.write('\n')
  else:
    with open('1.geofence.fen', 'a') as writer:
      writer.write('\t'.join(map(str,row)))
      writer.write('\n')


def write_poly(num, lat, lon):
  row = [str(lat), str(lon)]
  if int(num) == 0:
    with open('3.polygon.poly', 'w') as writer:
      writer.write('\t'.join(map(str,row)))
      writer.write('\n')
  else:
    with open('3.polygon.poly', 'a') as writer:
      writer.write('\t'.join(map(str,row)))
      writer.write('\n')


def write_wp(wp_num, wp_lat, wp_lon, wp_alt):
    row = [[str(wp_num), str(wp_lat), str(wp_lon), str(wp_alt)]]
    if int(wp_num) == 0:
        with open('waypoints.csv', 'w', newline='') as writeFile:
            writer = csv.writer(writeFile)
            # writer.writerows(head)
            writer.writerows(row)
    else:
        with open('waypoints.csv', 'a', newline='') as writeFile:
            writer = csv.writer(writeFile)
            writer.writerows(row)

    writeFile.close()


def write_obs(obs_num, obs_lat, obs_lon, obs_rad, obs_alt):
    head = [["No.","Lat","Lon","Radius","Altitude"]]
    row = [[str(obs_num), str(obs_lat), str(obs_lon), str(obs_rad), str(obs_alt)]]
    if int(obs_num) == 0:
        with open('obstacles.csv', 'w', newline='') as writeFile:
            writer = csv.writer(writeFile)
            writer.writerows(head)
            writer.writerows(row)
    else:
        with open('obstacles.csv', 'a', newline='') as writeFile:
            writer = csv.writer(writeFile)
            writer.writerows(row)

    writeFile.close()


# Read the waypoint.csv file for create the mavlink waypoint file
def reformat(gp_lat, gp_lon, ad_lat, ad_lon, elk_lat, elk_lon):
  wp = list(csv.reader(open('waypoints.csv', 'r'), delimiter=','))
  takeoff = 35.0
  dropheight = 35.0
  searchjoeheight = 90.0
  home_lat = 38.114843
  home_lon = -76.427984
  wp_num = 3
  num = 0
  while True:
    try:
      lat = wp[num][1]
      lon = wp[num][2]
      alt = wp[num][3]
      homerow = [0,1,0,16,0,0,0,0,home_lat,home_lon,1.3,1]
      takeoffrow = [1,0,3,22,0.0,0.0,0.0,0.0,0.0,0.0,takeoff,1]
      waypointrow = [wp_num,0,3,16,0.0,0.0,0.0,0.0,lat,lon,alt,1]
      airdroprow = [wp_num + 1,0,3,16,0.0,0.0,0.0,0.0,ad_lat,ad_lon,dropheight,1]
      joerow = [wp_num + 2,0,3,16,0.0,0.0,0.0,0.0,elk_lat,elk_lon,searchjoeheight,1]
      landingrow = [wp_num + 3,0,3,21,0.0,0.0,0.0,0.0,home_lat,home_lon,0,1]
      if int(wp_num) == 3:
        with open('2.SUAS2019.txt', 'w') as writer:
          writer.write('QGC WPL 110\n')
          writer.write('\t'.join(map(str,homerow)))  # Write home coordinate
          writer.write('\n')
          writer.write("\t".join(map(str,takeoffrow)))  # Write takeoff coordinate
          writer.write('\n')
          writer.write("\t".join(map(str,waypointrow)))  # Write first waypoint coordinates
          writer.write('\n')
      else:
        with open('2.SUAS2019.txt', 'a') as writer:
          writer.write("\t".join(map(str,waypointrow)))  # Write next waypoint coordinates
          writer.write('\n')
      wp_num += 1
      num += 1
    except:
      break
  with open('2.SUAS2019.txt', 'a') as writer:
    writer.write("\t".join(map(str,airdroprow)))  # Write Airdrop coordinate
    writer.write('\n')
    writer.write("\t".join(map(str,joerow)))  # Write emergentLastKnownPos coordinate
    writer.write('\n')
    writer.write("\t".join(map(str,landingrow)))  # Write Landing coordinate
    writer.write('\n')
  return 

# Create client object
client = client.Client(url='http://192.168.137.1:8000',
                       username='testuser',
                       password='testpass')

#Get missions
mission_id = int(input('Select mission ID:'))
mission = client.get_mission(mission_id)
json_mission = json_format.MessageToJson(mission)
# Use json library to read the mission data
json_parsed = json.loads(json_mission)
# json_parsed = json.loads(mission)

# Write flyzones altitude
altmax = json_parsed['flyZones'][0]['altitudeMax']
altmin = json_parsed['flyZones'][0]['altitudeMin']
print("Flyzone alt:", "Max", altmax, "Min", altmin)

# Write boundarypoints coordinates
bp_num = 0
while True:
    try:
        bp_lat = json_parsed['flyZones'][0]['boundaryPoints'][bp_num]['latitude']
        bp_lon = json_parsed['flyZones'][0]['boundaryPoints'][bp_num]['longitude']
        print("Boundarypoints:", bp_num + 1, bp_lat, bp_lon)
        write_bp(bp_num, bp_lat, bp_lon)
        bp_num += 1
    except:
        break

# Write searchGridPoints coordinate
gp_num = 0
while True:
    try:
      gp_lat = json_parsed['searchGridPoints'][gp_num]['latitude']
      gp_lon = json_parsed['searchGridPoints'][gp_num]['longitude']
      print("searchGridPoints:", gp_num, gp_lat, gp_lon)
      write_poly(gp_num, gp_lat, gp_lon)
      gp_num += 1
    except:
        break

# Write waypoints coordinates
wp_num = 0
while True:
  try:
    wp_lat = json_parsed['waypoints'][wp_num]['latitude']
    wp_lon = json_parsed['waypoints'][wp_num]['longitude']
    wp_alt = json_parsed['waypoints'][wp_num]['altitude']
    print("Waypoints:", wp_num + 1, wp_lat, wp_lon, wp_alt)
    write_wp(wp_num, wp_lat, wp_lon, wp_alt)
    wp_num += 1
  except:
    break

# Write airDropPos coordinates
ad_lat = json_parsed['airDropPos']['latitude']
ad_lon = json_parsed['airDropPos']['longitude']
print("airDropPos:", ad_lat, ad_lon)
# write_ad(ad_lat, ad_lon)

# Write emergentLastKnownPos coordinates
elk_lat = json_parsed['emergentLastKnownPos']['latitude']
elk_lon = json_parsed['emergentLastKnownPos']['longitude']
print("emergentLastKnownPos:", elk_lat, elk_lon)
# write_elk(elk_lat, elk_lon)

# Write emergentLastKnownPos coordinates
oax_lat = json_parsed['offAxisOdlcPos']['latitude']
oax_lon = json_parsed['offAxisOdlcPos']['longitude']
print("offAxisOdlcPos:", oax_lat, oax_lon)
# write_elk(elk_lat, elk_lon)

# Write stationaryObstacles coordinates
obs_num = 0
while True:
  try:
    obs_lat = json_parsed['stationaryObstacles'][obs_num]['latitude']
    obs_lon = json_parsed['stationaryObstacles'][obs_num]['longitude']
    obs_rad = json_parsed['stationaryObstacles'][obs_num]['radius']
    obs_alt = json_parsed['stationaryObstacles'][obs_num]['height']
    print("stationaryObstacles:", obs_num + 1, obs_lat, obs_lon, obs_rad)
    write_obs(obs_num, obs_lat, obs_lon, obs_rad, obs_alt)
    obs_num += 1
  except:
    break

# Reformat the mission file to a ready to use waypoint file
reformat(gp_lat, gp_lon, ad_lat, ad_lon, elk_lat, elk_lon)

# Restart?
done = str(input("Are you finished? (y/n)"))
if done == 'n':
  os.execl(sys.executable,sys.executable,*sys.argv)