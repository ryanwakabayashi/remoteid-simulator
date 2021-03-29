import argparse
import subprocess

latitude = 30.270083225404864
longitude = -97.7730248064704
orientation = 180


parser = argparse.ArgumentParser(description='Launch mult drones')
parser.add_argument('-n', '--count', help="Number of drones to be created.")
args = parser.parse_args()

# remote ID starting hex value
hex_string = "0x616263646566303132333435363738397778797A"
hex_int = int(hex_string, 16)

# this is setup for 4 vehicle generation in a square at 500 m apart
for vehicle_index in range(int(args.count)):
	subprocess.call(["python3", "mission_generator/mission_generator.py", "-n", str(vehicle_index), "-lat", str(latitude), "-long", str(longitude)])
	home_string = str(latitude) + "," + str(longitude) + ",0," + str(orientation)
	instance = "-I" + str(vehicle_index)
	remote_ID = hex(hex_int + vehicle_index)  # increment starting hex value by vehicle index
	subprocess.call(["gnome-terminal", "-x", "../ardupilot/build/sitl/bin/arducopter", "-S", instance, "--model", "+", "--speedup", "2", "--remoteid", remote_ID, "--home", home_string, "--defaults", "../ardupilot/Tools/autotest/default_params/copter.parm"])
	if vehicle_index == 0:
		latitude = latitude + 0.0045
	elif vehicle_index == 1:
		longitude = longitude + 0.0045
	elif vehicle_index == 2:
		latitude -= 0.0045
	orientation = orientation + 45


subprocess.call(["gnome-terminal", "--tab", "hold", "-x", "python3", "mult_drones.py", "-n", args.count])
