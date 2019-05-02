import csv
import time
import math


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


def write_wp(num, lat, lon):
    head = [["No.", "Lat", "Lon"]]
    row = [[str(num), str(lat), str(lon)]]
    if int(num) == 1:
        with open('new_point_test.csv', 'w', newline='') as writeFile:
            writer = csv.writer(writeFile)
            writer.writerows(head)
            writer.writerows(row)
    else:
        with open('new_point_test.csv', 'a', newline='') as writeFile:
            writer = csv.writer(writeFile)
            writer.writerows(row)

    writeFile.close()


# while True:
#     lat = input("Lat = ")
#     lon = input("Lon = ")
#     input_row = input("Row = ")
#     write_wp(input_row, lat, lon)


def new_way(lat, lon, input_row):
    file = open("new_point_test.csv", 'r+')
    newFile = []
    idNum = input_row
    for line in file:
        data = line.split(",")
        if data[0] == idNum:
            newLat = int(lat)
            newLon = int(lon)
            newLine = "%s, %s, %s\n" % (data[0], newLat, newLon)
            newFile.append(newLine)

        else:
            newFile.append(line)
    file.close()

    file = open("new_point_test.csv", 'w')

    for line in newFile:
        file.write(line)

    file.close()


# while True:
#     lat = input("Lat = ")
#     lon = input("Lon = ")
#     input_row = input("Row = ")
#     new_way(lat, lon, input_row)
#     #write_wp(input_row,lat,lon)

def write():
    csvData = [['No', 'Lat', 'Lon'], ['1', '35.111', '76.222'], ['2', '35.555', '76.222'], ['3', '35.444', '76.222']]

    with open('new_point_test.csv', 'w', newline='') as csvFile:
        writer = csv.writer(csvFile)
        writer.writerows(csvData)

    csvFile.close()


#  write()


def update_obs(select_row):
    while True:
        # Read the obstacle.csv file and store Lat,Lon,Radius in variables
        with open('obstacle.csv') as f:
            count_obs = sum(1 for line in f) - 1
        with open('obstacle.csv', 'r+') as obstacle_read:
            reader = csv.DictReader(obstacle_read)
            while True:
                for row in reader:
                    if int(row['No.']) == select_row:  # Check the existence of obstacle
                        obs_lat = float(row['Lat'])
                        obs_lon = float(row['Lon'])
                        obs_rad = float(row['Radius'])
                        obstacle_read.close()
                        return obs_lat, obs_lon, obs_rad, count_obs
                    elif int(row['No.']) == 0:
                        print("No obstacle detect")
                        obstacle_read.close()
                        time.sleep(1)
                        break
                break
        obstacle_read.close()


def obstacle_dis():
    while True:
        distance = []
        row = 1
        while True:
            cur_lat, cur_lon = 381447950, -764280950  # Check current position
            obs_lat, obs_lon, obs_rad, count_obs = update_obs(row)  # Check that the obstacle
            # Check distance between current position and obstacle shield
            distance.append(Haversine((cur_lon / 10000000, cur_lat / 10000000), (obs_lon, obs_lat)).meters)
            # /10000000 for convert int to float
            # if obs_lat != obs_lat1 or obs_lon != obs_lon1 or obs_rad != obs_rad1:
            #     return distance
            # print('Obstacle distance = %f' % distance[row])
            # time.sleep(0.5)
            row += 1
            if row > count_obs:
                min_dist = min(distance)
                return min_dist
        # if distance <= 60.0:  # FIXME: <------- should be adjust by vehicle velocity and object rad
        #     return distance


while True:
    #     start_row = 1
    #     write_wp(start_row)
    #     start_row += 1
    # obs_lat, obs_lon, obs_rad = update_obs()
    # print('Lat = %f\nLon = %f\nRad = %f\n' % (obs_lat, obs_lon, obs_rad))
    # time.sleep(1)
    # obs_lat, obs_lon, obs_rad = update_obs()
    # print('obs_lat = %f\nobs_lon = %f\nobs_rad = %f\n' % (obs_lat, obs_lon, obs_rad))
    start_time = time.time()
    print(obstacle_dis())
    print("--- %s seconds ---" % (time.time() - start_time))
    time.sleep(1)
