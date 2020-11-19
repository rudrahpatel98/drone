# Dependencies using dronekit library

from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import socket
import exceptions
import argparse

###Function definitions for mission####

##Function to connect script to drone
def connectMyCopter():
    parser = argparse.ArgumentParser(description='commands')
    parser.add_argument('--connect')
    args = parser.parse_args()

    connection_string = args.connect

    vehicle = connect(connection_string, wait_ready=True)

    return vehicle

##Function to arm the drone and takeoff into the air##
def arm_and_takeoff(targetAltitude):
    while not vehicle.is_armable:
        print("Waiting for vehicle to become armable")
        time.sleep(1)

    #Switch vehicle to GUIDED mode and wait for change
    vehicle.mode = VehicleMode("GUIDED")
    while vehicle.mode!="GUIDED":
        print("Waiting for vehicle to enter GUIDED mode")
        time.sleep(1)

    #Arm vehicle once GUIDED mode is confirmed
    vehicle.armed=True
    while vehicle.armed==False:
        print("Waiting for vehicle to become armed.")
        time.sleep(1)

    vehicle.simple_takeoff(targetAltitude)

    while True:
        print("Current Altitude: %d"%vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt>=targetAltitude*.95:
            break
        time.sleep(1)

    print("Reached target altitude")
    return None

#### Mission################################

vehicle=connectMyCopter()
print("About to takeoff..")

vehicle.mode=VehicleMode("GUIDED")
arm_and_takeoff(2) ##Tell drone to fly 2 meters in the sky
vehicle.mode=VehicleMode("LAND") ##Once drone reaches altitude, tell it to land

time.sleep(2)

print("End of function")

while True: 
    time.sleep(2)

vehicle.close()
### End of script
