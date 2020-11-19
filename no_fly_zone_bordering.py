# The drone programming course I bought on Udemy,
# from drone dojo, has a lot of resources to help 
# program the drone to this project's specifications.
# I've includeded source links to these, as they helped 
# me understand drone kit, and helped me write the 
# no_fly_zone_bordering.py program

# Dependencies using dronekit library
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time
import socket
import exceptions
import argparse

# Function definitions for mission

# Function to connect script to drone, source:
# https://community.dojofordrones.com/t/auto-mission-py/44
def connectMyCopter():
    parser = argparse.ArgumentParser(description='commands')
    parser.add_argument('--connect')
    args = parser.parse_args()

    connection_string = args.connect

    vehicle = connect(connection_string, wait_ready=True)

    return vehicle

# Function to arm the drone and takeoff into the air, source:
# https://community.dojofordrones.com/t/timeout-in-initialising-connection-error/68
def arm_and_takeoff(targetAltitude):
    while not vehicle.is_armable:
        print("Waiting for vehicle to become armable")
        time.sleep(1)

    # Switch vehicle to GUIDED mode and wait for change
    vehicle.mode = VehicleMode("GUIDED")
    while vehicle.mode!="GUIDED":
        print("Waiting for vehicle to enter GUIDED mode")
        time.sleep(1)

    # Arm vehicle once GUIDED mode is confirmed
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

    print("Target altitude reached")
    return None

# Function to send velocity command to drone via MavLink, sources:
# https://dronekit-python.readthedocs.io/en/latest/guide/copter/guided_mode.html
# https://github.com/tizianofiorenzani/how_do_drones_work/blob/master/scripts/02_control_with_arrow_keys.py
def set_velocity_body(vX,vY,vZ):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
            0,
            0,0,
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            0b0000111111000111, #-- BITMASK -> Consider only the velocities
            0, 0, 0, #--Position
            vX, vY, vZ, #--Velocity
            0, 0, 0, #--Accelerations
            0, 0)
    vehicle.send_mavlink(msg)
    vehicle.flush()


# Start of Mission:
vehicle=connectMyCopter()
print("About to takeoff..")

arm_and_takeoff(4) # 4m or 12ft

# moving NORTH, waypoint 1
counter=0
while counter<2:
    set_velocity_body(1,0,0)
    print("Direction: NORTH")
    time.sleep(1)
    counter+=1

time.sleep(1)

# moving back to original position
counter=0
while counter<2:
    set_velocity_body(-1,0,0)
    print("Direction: SOUTH")
    time.sleep(1)
    counter+=1

time.sleep(1)

# moving SOUTH, waypoint 2
counter=0
while counter<2:
    set_velocity_body(-1,0,0)
    print("Direction: SOUTH")
    time.sleep(1)
    counter+=1

time.sleep(1)

# moving back to original opsition
counter=0
while counter<2:
    set_velocity_body(1,0,0)
    print("Direction: NORTH")
    time.sleep(1)
    counter+=1

time.sleep(1)

# moving EAST, waypoint 3
counter=0
while counter<2:
    set_velocity_body(0,1,0)
    print("Direction: EAST")
    time.sleep(1)
    counter+=1

time.sleep(1)

# moving back to original position
counter=0
while counter<2:
    set_velocity_body(0,-1,0)
    print("Direction: WEST")
    time.sleep(1)
    counter+=1

time.sleep(1)

# moving WEST, waypoint 4
counter=0
while counter<2:
    set_velocity_body(0,-1,0)
    print("Direction: WEST")
    time.sleep(1)
    counter+=1
    
time.sleep(1)

# moving back to original position
counter=0
while counter<2:
    set_velocity_body(0,1,0)
    print("Direction: EAST")
    time.sleep(1)
    counter+=1

time.sleep(1)

# Return to launch
vehicle.mode = VehicleMode("RTL")


print("End of function")
print("Arducopter version: %s"%vehicle.version)

while True: 
    time.sleep(2)

vehicle.close()
# End of script
