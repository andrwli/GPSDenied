import dronekit
import math
import time

TARGET_WAYPOINT = dronekit.LocationGlobal(-35.356833, 149.162703, None)

# in lat/lon degrees
# 1 degree = 69 mi = 111045 m
DIST_MARGIN_OF_ERROR = 0.0003

TARGET_ATTITUDE = 100

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
        correct_yaw(vehicle, target_waypoint)
        reached_target = check_reached_target(vehicle, target_waypoint)

def correct_thrust(vehicle):
    if vehicle.location.alt <= TARGET_ATTITUDE:
        change_thrust(vehicle, 1500)
    else:
        change_thrust(vehicle, 1800)

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
    destination_angle_from_north = get_angle(current_waypoint, TARGET_WAYPOINT)
    heading_angle_from_north = get_heading_angle(vehicle)

    # angle to destination - negative angle from north
    error = destination_angle_from_north - (heading_angle_from_north - 360)
    increment = math.erf(error * P / 360) * max_yaw_increment

    print("error:", error)
    print("max_yaw_inc:", increment) 
    calculated_yaw = round(neutral_yaw + increment)

    return calculated_yaw

def get_current_location(vehicle):
    return vehicle.location.global_frame

def get_angle(current_waypoint, target_waypoint):
    lat_dist = target_waypoint.lat - current_waypoint.lat
    lon_dist = target_waypoint.lon - current_waypoint.lon

    return math.degrees(math.atan2(lon_dist, lat_dist))

def get_distance(p0, p1):
    return math.sqrt((p0.lat - p1.lat)**2 + (p0.lon - p1.lon)**2)

def get_heading_angle(vehicle):
    return vehicle.heading

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
