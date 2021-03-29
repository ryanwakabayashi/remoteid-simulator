#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
© Copyright 2015-2016, 3D Robotics.
mission_basic.py: Example demonstrating basic mission operations including creating, clearing and monitoring missions.

Full documentation is provided at http://python.dronekit.io/examples/mission_basic.html
"""
from __future__ import print_function
from enum import auto

from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
import time
import math
import os
import random
from pymavlink import mavutil
import array as arr


#Set up option parsing to get connection string
import argparse
parser = argparse.ArgumentParser(description='Demonstrates basic mission operations.')
parser.add_argument('--connect',
                   help="vehicle connection target string. If not specified, SITL automatically started and used.")
parser.add_argument('--mission', help="File location of mission to load.")
parser.add_argument('-n', '--count', help="Number of drones to create.")
args = parser.parse_args()

dronesCreated = int(args.count)
vehicles_string = []
vehicles = []

#each drone will run on port 5760 + (instance * 10)
for instance in range(dronesCreated):
	vehicles_string.append("tcp:127.0.0.1:" + str(5760 + (10 * instance)))


#implement connection and mission upload
#FIXME import_mission_filename is the file path where files will be generated
cwd = os.getcwd()
import_mission_filename = cwd + "/mission_generator/"
sitl = None

# Connect to the Vehicle
print('Connecting to vehicles')
for vehicle in vehicles_string:
    vehicles.append(connect(vehicle, wait_ready=True))



def get_location_metres(original_location, dNorth, dEast):
    """
    Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the
    specified `original_location`. The returned Location has the same `alt` value
    as `original_location`.

    The function is useful when you want to move the vehicle around specifying locations relative to
    the current vehicle position.
    The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.
    For more information see:
    http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
    """
    earth_radius=6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    return LocationGlobal(newlat, newlon,original_location.alt)


def get_distance_metres(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two LocationGlobal objects.

    This method is an approximation, and will not be accurate over large distances and close to the
    earth's poles. It comes from the ArduPilot test code:
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5


def distance_to_other_drone(drone_self):
    if vehicles[drone_index] == drone_self:
        return 0
    else:
        lat = vehicles[drone_index].location.global_relative_frame.lat
        lon = vehicles[drone_index].location.global_relative_frame.lon
        alt = vehicles[drone_index].location.global_relative_frame.alt
    target_drone = LocationGlobalRelative(lat, lon, alt)
    dist_to_drone = get_distance_metres(drone_self.location.global_frame, target_drone)
    return dist_to_drone


def download_mission():
    """
    Download the current mission from the vehicle.
    """
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready() # wait until download is complete.


def arm_and_takeoff(aTargetAltitude, vehicle):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print("Basic pre-arm checks")
    # Don't let the user try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)


    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: #Trigger just below target alt.
            print("Reached target altitude")
            break
        time.sleep(1)


def readmission(aFileName, vehicleNumber):
    """
    Load a mission from a file into a list.

    This function is used by upload_mission().
    """
    #print ("Reading mission from file: %s\n" % aFileName)
    cmds = vehicleNumber.commands
    missionlist=[]
    with open(aFileName) as f:
        for i, line in enumerate(f):
            if i==0:
                if not line.startswith('QGC WPL 110'):
                    raise Exception('File is not supported WP version')
            else:
                linearray=line.split('\t')
                ln_index=int(linearray[0])
                ln_currentwp=int(linearray[1])
                ln_frame=int(linearray[2])
                ln_command=int(linearray[3])
                ln_param1=float(linearray[4])
                ln_param2=float(linearray[5])
                ln_param3=float(linearray[6])
                ln_param4=float(linearray[7])
                ln_param5=float(linearray[8])
                ln_param6=float(linearray[9])
                ln_param7=float(linearray[10])
                ln_autocontinue=int(linearray[11].strip())
                cmd = Command( 0, 0, 0, ln_frame, ln_command, ln_currentwp, ln_autocontinue, ln_param1, ln_param2, ln_param3, ln_param4, ln_param5, ln_param6, ln_param7)
                missionlist.append(cmd)
    return missionlist


def upload_mission(aFileName, vehicle, index):
        """
        Upload a mission from a file.
        """
        #Read mission from file
        aFileName = import_mission_filename + "mission" + str(index) + ".txt"
        missionlist = readmission(aFileName, vehicle)

        print ("Upload mission from a file: %s" % aFileName)
        #Clear existing mission from vehicle
        print (' Clear mission')
        cmds = vehicle.commands
        cmds.clear()
        #Add new mission to vehicle
        for command in missionlist:
            cmds.add(command)
        print (' Upload mission')
        vehicle.commands.upload()


#upload_mission(import_mission_filename)
mission_index_load = 0
while mission_index_load < len(vehicles):
    upload_mission(import_mission_filename, vehicles[mission_index_load], mission_index_load)
    mission_index_load += 1


# From Copter 3.3 you will be able to take off using a mission item. Plane must take off using a mission item (currently).
#arm and takeoff for each drone one after the other
arm_and_takeoff_index = 0
while arm_and_takeoff_index < len(vehicles):
    arm_and_takeoff(10, vehicles[arm_and_takeoff_index])
    arm_and_takeoff_index += 1

print("Starting mission")


# Reset mission set to first (0) waypoint
#vehicle.commands.next=0
waypoint_0_index = 0
while waypoint_0_index < len(vehicles):
    vehicles[waypoint_0_index].commands.next=0
    waypoint_0_index += 1


# Set mode to AUTO to start mission
#vehicle.mode = VehicleMode("AUTO")
mode_index = 0
while mode_index < len(vehicles):
    vehicles[mode_index].mode = VehicleMode("AUTO")
    mode_index += 1


#FIXME may need to amend
#displays distance when drones are within a certain meter range of each other
while True:
    drone_index = 0
    while drone_index < len(vehicles):
        start = drone_index
        while start < len(vehicles):
            dist = distance_to_other_drone(vehicles[start])
            if dist != 0 and dist < 510: #this line limits the number of drones seen with dist in meters
                print('Distance from %s to drone %s: %.2f meters' %(drone_index, start, dist))
            start += 1
        drone_index += 1
    print('')
    time.sleep(4)

# print('Return to launch')
# vehicle.mode = VehicleMode("RTL")
land_index = 0
while land_index < len(vehicles):
    vehicles[land_index].mode = VehicleMode("RTL")
    land_index += 1


#Close vehicle object before exiting script
# print("Close vehicle object")
# vehicle.close()

# Shut down simulator if it was started.
# if sitl is not None:
#     sitl.stop()
