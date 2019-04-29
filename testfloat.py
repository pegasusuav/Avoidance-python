import csv


def get_guided_wp(point_no):
    with open('new_point.csv') as newpoint_read:
        reader = csv.DictReader(newpoint_read)
        for row in reader:
            if int(row['No.']) == point_no:
                LatZero=str(row['Lat'])
                LatNo=str(float(row['Lat']))
                LonZero=str(row['Lon'])
                LonNo=str(float(row['Lon']))
                
                go_lat = float(row['Lat'])
                go_lon = float(row['Lon'])
                if len(LatZero) != len(LatNo):
                    go_lat+=0.0000001
                if len(LonZero) != len(LonNo):
                    go_lon+=0.0000001
                point_no += 1
                print(type(row['Lon']))
                print("row['Lon']", row['Lon'])
                print("go_lat", go_lat)
                print("go_lon", go_lon)


point = 1
get_guided_wp(point)