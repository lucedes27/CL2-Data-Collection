import carla
import sys
import os

current_path = os.getcwd()
print("Current Path:", current_path)
sys.path.append('C:/Users/luced/Downloads/CARLA_0.9.14/WindowsNoEditor/PythonAPI/carla')
from agents.navigation.global_route_planner import GlobalRoutePlanner
from agents.navigation.controller import VehiclePIDController
from agents.navigation.local_planner import LocalPlanner

# from agents.navigation.global_route_planner_dao import GlobalRoutePlannerDAO

# Connect to CARLA server and get the world
client = carla.Client("localhost", 2000)
world = client.get_world()

# Get the map and the blueprint library
amap = world.get_map()
blueprint_library = world.get_blueprint_library()

# Choose a vehicle blueprint and spawn point
vehicle_blueprint = blueprint_library.filter('model3')[0]
spawn_points = world.get_map().get_spawn_points()
print("Spawn Points:", spawn_points)
a = carla.Location(spawn_points[50].location)
b = carla.Location(spawn_points[100].location)

# Spawn vehicle at point A
vehicle = world.spawn_actor(vehicle_blueprint, spawn_points[50])

# Add Global Router Planner
sampling_resolution = 2
grp = GlobalRoutePlanner(amap, sampling_resolution)
w1 = grp.trace_route(a, b)  # there are other functions can be used to generate a route in GlobalRoutePlanner.
print("Route:", w1)

# Visualize the route
i = 0
for w in w1:
    if i % 10 == 0:
        world.debug.draw_string(w[0].transform.location, 'O', draw_shadow=False,
                                color=carla.Color(r=255, g=0, b=0), life_time=120.0,
                                persistent_lines=True)
    else:
        world.debug.draw_string(w[0].transform.location, 'O', draw_shadow=False,
                                color=carla.Color(r=0, g=0, b=255), life_time=1000.0,
                                persistent_lines=True)
    i += 1

# Initialize Local Planner
local_planner = LocalPlanner(vehicle)
local_planner.set_global_plan(w1)

# Get final destination location
destination_location = w1[-1][0].transform.location

# Define threshold distance to destination
threshold = 10.0

# Run Local Planner until destination is reached
while True:
    # Apply control signal
    control_signal = local_planner.run_step(debug=False)
    vehicle.apply_control(control_signal)

    # Get current location of vehicle
    current_location = vehicle.get_location()

    # Calculate distance to destination
    distance = current_location.distance(destination_location)
    print("Distance to destination:", distance)

    # Check if destination is reached
    if distance < threshold:
        print("Destination Reached!")
        break

# Clean up
vehicle.destroy()

### TRAJECTORY-TRACKING EXAMPLE I FOUND ONLINE
# Spawn a vehicle at a
# vehicle_blueprint = client.get_world().get_blueprint_library().filter('model3')[0]
# spawn_point = spawn_points[50].transform
# spawn_point.location.z += 2
# vehicle = client.get_world().spawn_actor(vehicle_blueprint, spawn_point)
#
# custom_controller = VehiclePIDController(vehicle, args_lateral={'K_P': 1, 'K_D': 0.0, 'K_I': 0},
#                                          args_longitudinal={'K_P': 1, 'K_D': 0.0, 'K_I': 0.0})
#
# target_waypoint = spawn_points[100]
#
# ticks_to_track = 100
# for i in range(ticks_to_track):
#     control_signal = custom_controller.run_step(1, target_waypoint)
#     vehicle.apply_control(control_signal)
#
#
