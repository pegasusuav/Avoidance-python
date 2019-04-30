import csv
import time


def write_wp(num,lat,lon):
    row = [str(num), str(lat), str(lon)]

    with open('new_point_test.csv', 'r') as readFile:
        reader = csv.reader(readFile)
        lines = list(reader)
        lines[int(num)] = row

    with open('new_point_test.csv', 'w', newline = '') as writeFile:
        writer = csv.writer(writeFile)
        writer.writerows(lines)

    readFile.close()
    writeFile.close()

# while True:
#     lat = input("Lat = ")
#     lon = input("Lon = ")
#     start_row = input("Ros = ")
#     write_wp(start_row,lat,lon)
#     # start_row += 1

def write():
    csvData = [['No', 'Lat', 'Lon'], ['1', '35.111', '76.222'], ['2', '35.555', '76.222'], ['3', '35.444', '76.222']]

    with open('new_point_test.csv', 'w', newline='') as csvFile:
        writer = csv.writer(csvFile)
        writer.writerows(csvData)

    csvFile.close()

write()


def update_obs():
    while True:
        # Read the obstacle.csv file and store Lat,Lon,Radius in variables
        with open('obstacle.csv') as obstacle_read:
            reader = csv.DictReader(obstacle_read)
            # while True:
            for row in reader:
                # Check the existence of obstacle
                if row['No.'] != "":
                    obs_lat = float(row['Lat'])
                    obs_lon = float(row['Lon'])
                    obs_rad = float(row['Radius'])
                    return obs_lat, obs_lon, obs_rad
                else:
                    print("No obstacle found")
                    time.sleep(1)
                    continue


# while True:
#     start_row = 1
#     write_wp(start_row)
#     start_row += 1
    # obs_lat, obs_lon, obs_rad = update_obs()
    # print('Lat = %f\nLon = %f\nRad = %f\n' % (obs_lat, obs_lon, obs_rad))
    # time.sleep(1)
