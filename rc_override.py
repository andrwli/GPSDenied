'''
Module for Navigation with RC-override
Use move_to_target() to move to a target with rc_override

'''

import dronekit
import math
import time

#TARGET_WAYPOINT = dronekit.LocationGlobal(-35.356833, 149.162703, None)
TARGET_WAYPOINT = dronekit.LocationGlobal(-35.359124, 149.168475, None)

# in lat/lon degrees
# 1 degree = 69 mi = 111045 m
DIST_MARGIN_OF_ERROR = 0.0001

TARGET_RELATIVE_ALTITUDE = 50

# PID parameters
P = 3


def main():
    vehicle = dronekit.connect('tcp:127.0.0.1:5763', wait_ready=True)
    print(vehicle.heading)
    print(vehicle.location.global_frame)
    print(vehicle.location.global_relative_frame)
    print(vehicle.location.local_frame)
    arm(vehicle)
    move_to_target(vehicle, TARGET_WAYPOINT)
    vehicle.mode = dronekit.VehicleMode("LOITER")
    
    # Sleeping so all MAVLINK messages are correctly sent
    time.sleep(3)

def arm(vehicle):
    print("Basic pre-arm checks")
    # Don't let the user try to arm until autopilot is ready
    # If you need to disable the arming check,
    # just comment it with your own responsibility.
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    vehicle.mode = dronekit.VehicleMode("MANUAL")
    vehicle.armed = True

def move_to_target(vehicle, target_waypoint):
    reached_target = False
    
    while not reached_target:
        time.sleep(2.5)
        correct_thrust(vehicle)
        #change_thrust(vehicle, 1500)
        correct_yaw(vehicle, target_waypoint)

        # TODO: integrate multiprocessing to check if reached target
        reached_target = check_reached_target(vehicle, target_waypoint)

    return reached_target

def correct_thrust(vehicle):
    if vehicle.location.global_relative_frame.alt <= TARGET_RELATIVE_ALTITUDE:
        change_thrust(vehicle, 1700)
    else:
        change_thrust(vehicle, 1500)

def correct_yaw(vehicle, target_waypoint):
    calculated_yaw = get_calculated_yaw(vehicle, target_waypoint)
    change_yaw(vehicle, calculated_yaw) 

def get_calculated_yaw(vehicle, target_waypoint):
    '''
    Proportional controller

    '''
    neutral_yaw = 1500
    max_yaw_increment = 500
    calculated_yaw = 0 
    # calculate error btw heading and line to destination
    current_waypoint = get_current_location(vehicle)
    destination_angle_from_north = to_positive_angle(get_angle(current_waypoint, target_waypoint))
    heading_angle_from_north = get_heading_angle(vehicle)

    print("destination angle from north:", destination_angle_from_north)
    print("heading angle from north:", heading_angle_from_north)

    # angle to destination - negative angle from north
    error = get_smallest_angle_diff(heading_angle_from_north, destination_angle_from_north)
    error_percent = error * P / 360
    error_percent = -1 if error_percent < -1 else 1 if error_percent > 1 else error_percent
    #increment = math.erf(error * P / 360) * max_yaw_increment
    increment = error_percent * max_yaw_increment

    print("error:", error)
    print("max_yaw_inc:", increment) 
    calculated_yaw = round(neutral_yaw + increment)

    return calculated_yaw

def get_current_location(vehicle):
    '''
    Gets current lat, lon from GPS
    Need to replace implementation of this function for true GPS denied

    '''

    return vehicle.location.global_frame

def get_angle(current_waypoint, target_waypoint):
    lat_dist = target_waypoint.lat - current_waypoint.lat
    lon_dist = target_waypoint.lon - current_waypoint.lon

    return math.degrees(math.atan2(lon_dist, lat_dist))

def get_heading_angle(vehicle):
    return vehicle.heading

def get_distance(p0, p1):
    return math.sqrt((p0.lat - p1.lat)**2 + (p0.lon - p1.lon)**2)

def to_positive_angle(angle):
    return angle % 360

def get_smallest_angle_diff(source, target):
    result = target - source
    result = (result + 180) % 360 - 180

    return result

def circle_currrent_location(vehicle):
    '''
    Circle location in GPS-denied.
    Should use loiter over this function.

    '''

    change_thrust(vehicle, 1050)
    vehicle.mode = dronekit.VehicleMode("CIRCLE")

def check_reached_target(vehicle, target_waypoint):
    current_waypoint = get_current_location(vehicle)
    distance = get_distance(target_waypoint, current_waypoint)
    
    if abs(distance) < DIST_MARGIN_OF_ERROR:
        return True
    
    return False

def change_yaw(vehicle, yaw):
    print("yaw:", yaw)
    vehicle.channels.overrides['4'] = yaw

def change_thrust(vehicle, thrust):
    print("thrust: ", thrust)
    vehicle.channels.overrides['3'] = thrust

if __name__ == '__main__':
    main()
