import carla
import casadi as ca
import numpy as np
import matplotlib.pyplot as plt
import datetime
import os

from boxconstraint import BoxConstraint

SIM_DURATION = 200  # Simulation duration in time steps

## SETUP ##
# Connect to CARLA
client = carla.Client('localhost', 2000)
world = client.get_world()
mymap = world.get_map()
spectator = world.get_spectator()

# CARLA Settings
settings = world.get_settings()
settings.synchronous_mode = True  # Enables synchronous mode
TIME_STEP = 0.05  # Time step for synchronous mode
settings.fixed_delta_seconds = TIME_STEP
settings.substepping = True
settings.max_substep_delta_time = 0.01
settings.max_substeps = 10
world.apply_settings(settings)

# Output client and world objects to console
print(client)
print(world)


# Function to move the spectator camera
def move_spectator_to_vehicle(vehicle, spectator, distance=5):
    vehicle_location = vehicle.get_location()
    # Set viewing angle to slightly above the vehicle
    spectator_transform = carla.Transform(vehicle_location + carla.Location(z=distance), carla.Rotation(pitch=-90))
    spectator.set_transform(spectator_transform)


# Use recommended spawn points
spawn_points = mymap.get_spawn_points()
spawn_point = spawn_points[0]

# Spawn vehicle
vehicles = world.get_actors().filter('vehicle.*')
blueprint_library = world.get_blueprint_library()
vehicle_bp = blueprint_library.filter('model3')[0]
if len(vehicles) == 0:
    vehicle = world.spawn_actor(vehicle_bp, spawn_point)
else:
    # Reset world
    for vehicle in vehicles:
        vehicle.destroy()
    vehicle = world.spawn_actor(vehicle_bp, spawn_point)
print(vehicle)

# Get spawn coordinates and orientation
x_spawn = spawn_point.location.x
y_spawn = spawn_point.location.y
theta_spawn = spawn_point.rotation.yaw / 180 * ca.pi


def generate_waypoint_relative_to_spawn(forward_offset=0, sideways_offset=0):
    # Positive forward_offset moves the waypoint in front of the vehicle
    # Positive (input) sideways_offset moves the waypoint to the right of the vehicle
    sideways_offset = -sideways_offset
    return ca.vertcat(
        x_spawn + forward_offset * ca.cos(theta_spawn) + sideways_offset * ca.sin(theta_spawn),
        y_spawn + forward_offset * ca.sin(theta_spawn) - sideways_offset * ca.cos(theta_spawn)
    )


waypoints = []

for i in range(1000):
    waypoints.append(generate_waypoint_relative_to_spawn(10, 0))

# Parameters
params = {'L': 2.5}  # Wheelbase of the vehicle
N = 20  # Prediction horizon for optimization prbolem
dt = TIME_STEP  # Time step for discretization
state_dim = 4  # Dimension of the state [x, y, theta, v]
control_dim = 2  # Dimension of the control input [steering angle, acceleration]

# Initialize Opti object
opti = ca.Opti()

# Declare variables
X = opti.variable(state_dim, N + 1)  # state trajectory variables over prediction horizon
U = opti.variable(control_dim, N)  # control trajectory variables over prediction horizon
P = opti.parameter(state_dim)  # initial state parameter
Q = ca.MX.eye(state_dim)  # state penalty matrix for objective function
R = 0.1 * ca.MX.eye(control_dim)  # control penalty matrix for objective function
W = opti.parameter(2, N)  # Reference trajectory parameter

# Objective
obj = 0
for k in range(N):
    x_k = X[:, k]  # Current state
    u_k = U[:, k]  # Current control input
    x_next = X[:, k + 1]  # Next state

    x_ref = ca.vertcat(W[:, k],
                       ca.MX.zeros(state_dim - 2, 1))  # Reference state with waypoint and zero for other states

    dx = x_k - x_ref  # Deviation of state from reference state
    du = u_k  # Control input deviation (assuming a desired control input of zero)

    # Quadratic cost with reference state and control input
    obj += ca.mtimes([ca.mtimes(dx.T, Q), dx]) + ca.mtimes(
        [ca.mtimes(du.T, R), du])  # Minimize quadratic cost and deviation from reference state

opti.minimize(obj)

# Maximum steerin angle for dynamics
max_steering_angle_deg = 70  # Maximum steering angle in degrees
max_steering_angle_rad = max_steering_angle_deg * (ca.pi / 180)  # Maximum steering angle in radians

# Dynamics (Euler discretization using bicycle model)
for k in range(N):
    steering_angle_rad = U[0, k] * max_steering_angle_rad  # Convert normalized steering angle to radians
    opti.subject_to(X[:, k + 1] == X[:, k] + dt * ca.vertcat(
        X[3, k] * ca.cos(X[2, k]),
        X[3, k] * ca.sin(X[2, k]),
        X[3, k] / params['L'] * ca.tan(steering_angle_rad),
        U[1, k]
    ))

# Constraints
opti.subject_to(X[:, 0] == P)  # Initial state constraint

# Input constraints
steering_angle_bounds = [-1.0, 1.0]
acceleration_bounds = [-1.0, 1.0]
lb = np.array([steering_angle_bounds[0], acceleration_bounds[0]]).reshape(-1, 1)
ub = np.array([steering_angle_bounds[1], acceleration_bounds[1]]).reshape(-1, 1)
action_space = BoxConstraint(lb=lb, ub=ub)

# Apply constraints to optimization problem
for i in range(N):
    # Input constraints
    opti.subject_to(action_space.H_np @ U[:, i] <= action_space.b_np)

# Setup solver
acceptable_dual_inf_tol = 1e11
acceptable_compl_inf_tol = 1e-3
acceptable_iter = 10
acceptable_constr_viol_tol = 1e-3
acceptable_tol = 1e4

opts = {"ipopt.acceptable_tol": acceptable_tol,
        "ipopt.acceptable_constr_viol_tol": acceptable_constr_viol_tol,
        "ipopt.acceptable_dual_inf_tol": acceptable_dual_inf_tol,
        "ipopt.acceptable_iter": acceptable_iter,
        "ipopt.acceptable_compl_inf_tol": acceptable_compl_inf_tol,
        "ipopt.hessian_approximation": "limited-memory",
        "ipopt.print_level": 0}
opti.solver('ipopt', opts)

# Array to store closed-loop trajectory states (X and Y coordinates)
closed_loop_trajectory = []

# Initialize warm-start parameters
prev_sol_x = None
prev_sol_u = None

# Clear debug_plots folder
for filename in os.listdir('debug_plots'):
    os.remove('debug_plots/' + filename)

# Main Loop
for i in range(SIM_DURATION - N):
    print("Iteration: ", i)

    move_spectator_to_vehicle(vehicle, spectator)

    #  Fetch initial state from CARLA
    x0 = vehicle.get_transform().location.x
    y0 = vehicle.get_transform().location.y
    theta0 = vehicle.get_transform().rotation.yaw / 180 * ca.pi
    velocity_vector = vehicle.get_velocity()
    v0 = ca.sqrt(velocity_vector.x ** 2 + velocity_vector.y ** 2)

    # Append current state
    if i > 0:
        closed_loop_trajectory.append([x0, y0])

    # Set initial state for optimization problem
    initial_state = ca.vertcat(x0, y0, theta0, v0)
    opti.set_value(P, initial_state)

    # Set the reference trajectory for the current iteration
    opti.set_value(W, ca.horzcat(*waypoints[i:i + N]))  # Concatenate waypoints

    if prev_sol_x is not None and prev_sol_u is not None:
        # Warm-starting the solver with the previous solution
        opti.set_initial(X, prev_sol_x)
        opti.set_initial(U, prev_sol_u)

    # Solve the optimization problem
    sol = opti.solve()

    # If the solver is successful, apply the first control input to the vehicle
    if sol.stats()['success']:
        u = sol.value(U[:, 0])
        if u[1] < 0:
            vehicle.apply_control(carla.VehicleControl(throttle=u[1], steer=u[0], reverse=True))
        else:
            vehicle.apply_control(carla.VehicleControl(throttle=u[1], steer=u[0]))

        # Update previous solution variables for warm-starting next iteration
        prev_sol_x = sol.value(X)
        prev_sol_u = sol.value(U)

        # Plot open-loop trajectory
        fig, ax = plt.subplots(1, 1)
        ax.set_xlabel('x')
        ax.set_ylabel('y')

        # Plot spawn point and arrow for spawn orientation
        ax.plot(x_spawn, y_spawn, 'bo')
        ax.arrow(x_spawn, y_spawn, 0.5 * ca.cos(theta_spawn), 0.5 * ca.sin(theta_spawn), width=0.1)

        # Plot current state and goal state
        ax.plot(x0, y0, 'go')
        ax.plot(waypoints[i][0], waypoints[i][1], 'ro')

        # Plot open-loop trajectory
        ax.plot(opti.debug.value(X)[0, :], opti.debug.value(X)[1, :], 'b-')

        # Plot closed-loop trajectory
        ax.plot([x[0] for x in closed_loop_trajectory], [x[1] for x in closed_loop_trajectory], 'g-')

        # Display cost
        ax.text(0.1, 0.9, "Cost: {:.2f}".format(sol.value(obj)), transform=ax.transAxes)

        plt.savefig("debug_plots/{}_{}.png".format(i, datetime.datetime.now()))
        plt.close(fig)
    else:
        print("Error in optimization problem.")

    world.tick()  # Tick the CARLA world

# Store closed-loop trajectory in csv file
closed_loop_trajectory = np.array(closed_loop_trajectory)
np.savetxt("closed_loop_trajectory.csv", closed_loop_trajectory, delimiter=",")

print("Done.")
