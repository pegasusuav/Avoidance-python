import csv
import time


def write_wp(num,lat,lon):
    head = [["No.", "Lat", "Lon"]]
    row = [[str(num), str(lat), str(lon)]]
    if int(num) == 1:
        with open('new_point_test.csv', 'w', newline = '') as writeFile:
            writer = csv.writer(writeFile)
            writer.writerows(head)
            writer.writerows(row)
    else:
        with open('new_point_test.csv', 'a', newline = '') as writeFile:
            writer = csv.writer(writeFile)
            writer.writerows(row)

    writeFile.close()

while True:
    lat = input("Lat = ")
    lon = input("Lon = ")
    input_row = input("Row = ")
    write_wp(input_row, lat, lon)


def new_way(lat, lon, input_row):
    file = open("new_point_test.csv", 'r')
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
