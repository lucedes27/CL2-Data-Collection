import carla
import sys
import os
import shutil
import matplotlib.pyplot as plt

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

# Clear all actors in the world
for actor in world.get_actors():
    actor.destroy()

# Choose a vehicle blueprint and spawn point
vehicle_blueprint = blueprint_library.filter('model3')[0]
spawn_points = world.get_map().get_spawn_points()
print("Spawn Points:", spawn_points)
a = carla.Location(spawn_points[50].location)
b = carla.Location(spawn_points[100].location)

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

# Get final destination location
destination_location = w1[-1][0].transform.location

# Define threshold distance to destination
threshold = 10.0

# Set up folder for trajectory plots
folder_name = 'trajectory_plots'
if not os.path.exists(folder_name):
    os.makedirs(folder_name)

# Clear debug_plots folder
for file in os.listdir(folder_name):
    file_path = os.path.join(folder_name, file)
    try:
        if os.path.isfile(file_path):
            os.unlink(file_path)
    except Exception as e:
        print(e)

# Clear all subfolders in debug_plots folder
for folder in os.listdir(folder_name):
    folder_path = os.path.join(folder_name, folder)
    try:
        if os.path.isdir(folder_path):
            shutil.rmtree(folder_path)
    except Exception as e:
        print(e)

# Set up the friction values to test
friction_values = [0.1, 0.2, 0.3, 0.4, 0.5,
                   0.6, 0.7, 0.8, 0.9, 1.0]


# Function to plot and save trajectory
def plot_and_save_trajectory(planned_path, actual_path, friction_value):
    plt.figure()
    planned_x, planned_y = zip(*[(wp[0].transform.location.x, wp[0].transform.location.y) for wp in planned_path])
    actual_x, actual_y = zip(*actual_path)
    plt.plot(planned_x, planned_y, 'r-', label='Planned Path')
    plt.plot(actual_x, actual_y, 'b-', label='Actual Path')
    plt.title(f'Trajectory with Friction Value: {friction_value}')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.legend()
    plt.savefig(f'{folder_name}/trajectory_friction_{friction_value}.png')
    plt.close()


# Main loop for different friction values
for friction_value in friction_values:
    print(f"Friction Value: {friction_value}")

    # Spawn vehicle at point A
    vehicle = world.spawn_actor(vehicle_blueprint, spawn_points[50])

    # Set wheel friction
    for wheel in vehicle.get_physics_control().wheels:
        wheel.tire_friction = friction_value
    vehicle.apply_physics_control(vehicle.get_physics_control())

    # Initialize Local Planner
    local_planner = LocalPlanner(vehicle)
    local_planner.set_global_plan(w1)
    local_planner.follow_speed_limits(False)
    local_planner.set_speed(20)

    # Run Local Planner until destination is reached
    actual_path = []
    while True:
        control_signal = local_planner.run_step(debug=False)
        vehicle.apply_control(control_signal)

        # Store current location
        actual_path.append((vehicle.get_location().x, vehicle.get_location().y))

        # Check if destination is reached
        if vehicle.get_location().distance(destination_location) < threshold:
            break

    # Plot and save trajectory
    plot_and_save_trajectory(w1, actual_path, friction_value)

    # Clean up
    vehicle.destroy()

print("Simulation completed.")

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
