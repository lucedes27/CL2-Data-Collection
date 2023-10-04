from casadi import SX, vertcat
import casadi as ca
import time
import carla

## SETUP ##
# Connect to CARLA
client = carla.Client('localhost', 2000)
world = client.get_world()
mymap = world.get_map()
spectator = world.get_spectator()

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


## MPC SETUP ##
def bicycle_model(state, controls, params):
    x, y, theta, v = state[0], state[1], state[2], state[3]
    phi, a = controls[0], controls[1]
    L = params['L']

    dx = v * SX.cos(theta)
    dy = v * SX.sin(theta)
    dtheta = v / L * SX.tan(phi)
    dv = a

    return vertcat(dx, dy, dtheta, dv)


def euler_discretization(model, state, controls, params, dt):
    return state + dt * model(state, controls, params)


# Parameters
params = {'L': 2.5}
N = 20  # Prediction horizon
dt = 0.1  # Time step
state_dim = 4
control_dim = 2

# Symbolic variables
U = ca.MX.sym('U', N * control_dim)  # Control trajectory over the horizon
X = ca.MX.sym('X', state_dim)  # Initial state
P = ca.MX.sym('P', N * state_dim)  # Waypoints over the horizon

# Objective and constraints
Q = ca.MX.eye(state_dim)  # State penalty matrix
R = ca.MX.eye(control_dim)  # Control penalty matrix

# Initialize objective and constraints
obj = 0
g = []
X_k = X

# Loop over each step in the prediction horizon to build objective and constraints
for k in range(N):
    u_k = U[k * control_dim:(k + 1) * control_dim]
    p_k = P[k * state_dim:(k + 1) * state_dim]

    # Add to objective
    obj += ca.mtimes([(X_k - p_k).T, Q, X_k - p_k]) + ca.mtimes([u_k.T, R, u_k])

    # Update state (Euler discretization)
    X_k = euler_discretization(bicycle_model, X_k, u_k, params, dt)
    g.append(X_k)

lb_U = ca.MX([-0.5] * N + [-1.0] * N)  # Lower bounds for [steer, throttle]
ub_U = ca.MX([0.5] * N + [1.0] * N)  # Upper bounds for [steer, throttle]

# Optimization problem
opts = {'ipopt.print_level': 0, 'print_time': 0}
nlp = {'x': U, 'f': obj, 'g': ca.vertcat(*g), 'p': ca.vertcat(X, P)}
solver = ca.nlpsol('solver', 'ipopt', nlp, opts)

lb_g = ca.MX([-ca.inf]*state_dim*N)
ub_g = ca.MX([ca.inf]*state_dim*N)

# Set up waypoints to form a circle around spawn point
# waypoints = []
# for i in range(N):
#     angle = i * 2 * ca.pi / N
#     x = 5 * ca.cos(angle)
#     y = 5 * ca.sin(angle)
#     waypoints.append(ca.vertcat(x, y, 0, 0))

# Initialize spectator camera
move_spectator_to_vehicle(vehicle, spectator)

# Loop to have the vehicle follow the waypoints using Model Predictive Control
while True:
    # Get the current vehicle state
    x0 = vehicle.get_transform().location.x
    y0 = vehicle.get_transform().location.y
    theta0 = vehicle.get_transform().rotation.yaw / 180 * ca.pi
    v0 = vehicle.get_velocity().x

    print(f"Current state: x={x0}, y={y0}, theta={theta0}, v={v0}")

    # Initialize an initial guess for controls
    initial_guess = ca.DM.zeros(N * control_dim)

    print(f"Initial guess for controls: {initial_guess}")

    # Initial state
    initial_state = ca.vertcat(x0, y0, theta0, v0)

    print(f"Initial state: {initial_state}")

    # Calculate the single waypoint in front of the car
    waypoint_distance = 2.0  # Distance ahead of the car for the waypoint
    x_waypoint = x0 + waypoint_distance * ca.cos(theta0)
    y_waypoint = y0 + waypoint_distance * ca.sin(theta0)
    single_waypoint = ca.vertcat(x_waypoint, y_waypoint, 0, 0)
    waypoints = [single_waypoint] * N

    print(f"Waypoint: {single_waypoint}")

    # Concatenate initial_state and waypoints as the p parameter
    p = ca.vertcat(initial_state, ca.reshape(ca.vertcat(*waypoints), -1, 1))

    print(f"Parameter vector p: {p}")

    # Solve the optimization problem
    sol = solver(x0=initial_guess, p=p)

    if sol['f'] is not None:
        print("Solver found a solution.")
        print(f"Objective value: {float(sol['f'])}")

        controls = sol['x'].full().flatten()

        print(f"Optimal controls: {controls}")

        # Extract the first control and normalize it if needed
        throttle = float(controls[1])
        steer = float(controls[0])

        print(f"Throttle: {throttle}, Steer: {steer}")

        # Apply the first control input to the vehicle
        vehicle.apply_control(carla.VehicleControl(throttle=throttle, steer=steer))

        print("Control applied to vehicle.")

        # Move the spectator camera
        move_spectator_to_vehicle(vehicle, spectator)

        print("Spectator camera moved.")
    else:
        print("Solver did not find a solution.")

    # Sleep for a bit
    time.sleep(dt)
    print("Sleeping for next iteration.\n")
