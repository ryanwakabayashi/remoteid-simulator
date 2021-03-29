import argparse
import random
import os


parser = argparse.ArgumentParser(description="Generate missions with 5 waypoints")
parser.add_argument("-v", "--version", help="show program version", action="store_true")
parser.add_argument("-n", "--count", help="Mission generator index")
parser.add_argument("-lat", "--latitude", help="Starting home latitude.")
parser.add_argument("-long", "--longitude", help="Starting home longitude.")

args = parser.parse_args()

if args.version:
    print("Version 1.0")
elif args.count:
    index = int(args.count)
    curr_latitude = float(args.latitude)
    curr_longitude = float(args.longitude)

name = "mission"
# for all in range(nmissions):
mission_name = "./mission_generator/" + name + str(index) + ".txt"
mission_file = open(mission_name, "w+")

#first two lines of mission.txt files
mission_file.write("QGC WPL 110\n")
mission_file.write("0\t0\t0\t16\t0.000000\t0.000000\t0.000000\t0.000000\t" + str(curr_latitude) + "\t" + str(curr_longitude) + "\t600\t1\n")


entry = 1
#make 5 waypoints
for five in range(5):
    
    #generate random latitude, longitude, altitude
    rand = random.randint(-9999, 9999) / 1000000
    latitude = curr_latitude + rand
    rand = random.randint(-9999, 9999) / 1000000
    longitude = curr_longitude + rand
    #121 meters is approx 400 ft for max height
    altitude = random.randint(40, 121)

    #write change speed to file
    airSpeed = random.randint(1,40)
    mission_file.write(str(entry) + "\t0\t0\t178\t0.000000\t" + str(airSpeed) + ".0000\t0.000000\t0.000000\t0.000000\t0.000000\t0.000000\t1\n")
    entry += 1
    
    #write waypoint to file
    mission_file.write(str(entry) + "\t0\t3\t16\t0.000000\t0.000000\t0.000000\t0.000000\t{:.6f}\t{:.6f}\t{:.5f}\t1\n".format(latitude, longitude, altitude))
    entry += 1
    
    
    
mission_file.close()

print("Generated mission: %s" %args.count)
