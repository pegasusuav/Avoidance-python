import csv
import time

def write_wp(s_row):
    while True:
        row = ['2', 'A', 'B']

        with open('new_point_test.csv', 'r') as readFile:
            reader = csv.reader(readFile)

        lines = list(reader)
        print(lines)
        lines[s_row] = row
        readFile.close()    
        
        with open('new_point_test.csv', 'w') as writeFile:
            writer = csv.writer(writeFile)
            writer.writerows(lines)

        writeFile.close()
        print('Planning done\n')
        return


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


while True:
    # start_row = 1
    # write_wp(start_row)
    # start_row += 1
    obs_lat, obs_lon, obs_rad = update_obs()
    print('Lat = %f\nLon = %f\nRad = %f\n' % (obs_lat, obs_lon, obs_rad))
    time.sleep(1)
