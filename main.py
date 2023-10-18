from casadi import SX, vertcat
import casadi as ca
import time
import carla
import csv

## SETUP ##
# Connect to CARLA
client = carla.Client('localhost', 2000)
world = client.get_world()
mymap = world.get_map()
spectator = world.get_spectator()

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
if len(vehicles) == 0:
    blueprint_library = world.get_blueprint_library()
    vehicle_bp = blueprint_library.filter('model3')[0]
    vehicle = world.spawn_actor(vehicle_bp, spawn_point)
    # vehicle.set_autopilot(True)
else:
    vehicle = vehicles[0]
print(vehicle)

# Parameters
params = {'L': 2.5}  # Wheelbase of the vehicle
N = 20  # Prediction horizon for optimization prbolem
dt = 0.1  # Time step for discretization
state_dim = 4  # Dimension of the state [x, y, theta, v]
control_dim = 2  # Dimension of the control input [steering angle, acceleration]

# Initialize Opti object
opti = ca.Opti()

# Declare variables
X = opti.variable(state_dim, N + 1)  # state trajectory variables over prediction horizon
U = opti.variable(control_dim, N)  # control trajectory variables over prediction horizon
P = opti.parameter(state_dim)  # initial state parameter
Q = ca.MX.eye(state_dim)  # state penalty matrix for objective function
R = ca.MX.eye(control_dim)  # control penalty matrix for objective function
W = opti.parameter(2)  # Waypoint parameter

# Objective
obj = 0
for k in range(N):
    x_k = X[:, k]  # Current state
    u_k = U[:, k]  # Current control input
    x_next = X[:, k + 1]  # Next state

    x_ref = ca.vertcat(W, ca.MX.zeros(state_dim - 2, 1))  # Reference state with waypoint and zero for other states

    dx = x_k - x_ref  # Deviation of state from reference state
    du = u_k  # Control input deviation (assuming a desired control input of zero)

    # Quadratic cost with reference state and control input
    obj += ca.mtimes([ca.mtimes(dx.T, Q), dx]) + ca.mtimes(
        [ca.mtimes(du.T, R), du])  # Minimize quadratic cost and deviation from reference state

opti.minimize(obj)

# Dynamics (Euler discretization using bicycle model)
for k in range(N):
    opti.subject_to(X[:, k + 1] == X[:, k] + dt * ca.vertcat(
        X[3, k] * ca.cos(X[2, k]),
        X[3, k] * ca.sin(X[2, k]),
        X[3, k] / params['L'] * ca.tan(U[0, k]),
        U[1, k]
    ))

# Initial constraints
opti.subject_to(X[:, 0] == P)

# Input constraints
opti.subject_to(opti.bounded(-0.5, U[0, :], 0.5))  # Steering angle
opti.subject_to(opti.bounded(-1.0, U[1, :], 1.0))  # Acceleration

# Setup solver
opts = {'ipopt.print_level': 0, 'print_time': 0}  # Solver options for ipopt
opti.solver('ipopt', opts)

# Initialize warm-start parameters
prev_sol_x = None
prev_sol_u = None

# Main Loop
for i in range(10):
    move_spectator_to_vehicle(vehicle, spectator)

    #  Fetch initial state from CARLA
    x0 = vehicle.get_transform().location.x
    y0 = vehicle.get_transform().location.y
    theta0 = vehicle.get_transform().rotation.yaw / 180 * ca.pi
    v0 = vehicle.get_velocity().x

    # Set initial state for optimization problem
    initial_state = ca.vertcat(x0, y0, theta0, v0)
    opti.set_value(P, initial_state)

    # Calculate the waypoint in front of the car
    waypoint = ca.vertcat(x0 + 10 * ca.cos(theta0), y0 + 10 * ca.sin(theta0))
    opti.set_value(W, waypoint)

    if prev_sol_x is not None and prev_sol_u is not None:
        # Warm-starting the solver with the previous solution
        opti.set_initial(X, prev_sol_x)
        opti.set_initial(U, prev_sol_u)

    # Solve the optimization problem
    sol = opti.solve()

    # If the solver is successful, apply the first control input to the vehicle
    if sol.stats()['success']:
        u = sol.value(U[:, 0])
        vehicle.apply_control(carla.VehicleControl(throttle=u[1], steer=u[0]))

        # Update previous solution variables for warm-starting next iteration
        prev_sol_x = sol.value(X)
        prev_sol_u = sol.value(U)
    else:
        print("Error in optimization problem.")

    time.sleep(dt)

print("Done.")
