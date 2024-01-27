import carla
import sys
import os

current_path = os.getcwd()
print("Current Path:", current_path)
sys.path.append('C:/Users/luced/Downloads/CARLA_0.9.14/WindowsNoEditor/PythonAPI/carla')
from agents.navigation.global_route_planner import GlobalRoutePlanner
from agents.navigation.controller import VehiclePIDController

# from agents.navigation.global_route_planner_dao import GlobalRoutePlannerDAO

client = carla.Client("localhost", 2000)
world = client.get_world()
amap = world.get_map()
sampling_resolution = 2
# dao = GlobalRoutePlannerDAO(amap, sampling_resolution)
grp = GlobalRoutePlanner(amap, sampling_resolution)
# grp.setup()
spawn_points = world.get_map().get_spawn_points()
print(spawn_points)
a = carla.Location(spawn_points[50].location)
b = carla.Location(spawn_points[100].location)
w1 = grp.trace_route(a, b)  # there are other funcations can be used to generate a route in GlobalRoutePlanner.
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
