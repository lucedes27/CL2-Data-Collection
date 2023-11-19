# CL2 Data Collection

## Running the script
CARLA server needs to be running in the background for the data collection to work.

Running the script on lab computer (E5-5044):
- Navigate to CARLA directory in Rishi's directory.
- Launch CARLA using ```make launch```
- After watiing for Unity Engine to load, press 'Play' button in Unity Engine.
- Run ```main.py``` (in PyCharm or terminal)

## Script Functionality

The `main.py` script performs the following key functions:

1. **CARLA Connection**: It connects to a running instance of the CARLA simulator, allowing interaction with the virtual environment.

2. **Vehicle Setup**: The script spawns a Tesla Model 3 (or any other specified vehicle) in the CARLA environment at a predefined spawn point.

3. **Waypoint Generation**: A series of waypoints are generated relative to the vehicle's spawn point. These waypoints dictate the path the vehicle will attempt to follow.

4. **Model Predictive Control (MPC)**: The core of the script is an MPC setup. This includes defining the optimization problem where the vehicle's trajectory is optimized to follow the generated waypoints. The optimization problem uses a bicycle model for vehicle dynamics, constraints on control inputs (steering and acceleration), and a cost function that penalizes deviation from the desired trajectory and control effort.

5. **Solver Configuration**: IPOPT, an interior point optimizer, is configured with specific tolerances and parameters to solve the optimization problem at each time step.

6. **Data Collection**: During the simulation, the script collects data on the planned trajectory (open-loop data), the actual trajectory followed by the vehicle (closed-loop data), and the residuals (differences between the predicted and actual states).

7. **Visualization and Debugging**: The script generates plots for each iteration, showing the planned and actual trajectories, which are helpful for debugging and analysis. These plots are saved in a specified directory for later review.

8. **Data Storage**: At the end of the simulation, the collected data (closed-loop, open-loop, and residuals) are saved to CSV files for further analysis.


## Output

Upon successful completion of the simulation, the script will generate and save the following:
- A series of plots visualizing the vehicle's trajectory over time.
- CSV files containing detailed data on the vehicle's motion and control inputs:
  - `closed_loop_data.csv`: Data on the actual trajectory followed by the vehicle.
  - `open_loop_data.csv`: Data on the planned trajectory as computed by the MPC.
  - `residuals_data.csv`: Data on the residuals, highlighting the differences between the planned and actual trajectories.
