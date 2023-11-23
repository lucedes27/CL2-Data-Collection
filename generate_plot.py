from matplotlib import pyplot as plt
import casadi as ca


def generate_plot_outer(spawn_point, x0, y0, theta0, waypoints, i, u, X, opti, obj, sol, closed_loop_data):
    # Plot open-loop trajectory
    fig, ax = plt.subplots(1, 1)
    ax.set_xlabel('x')
    ax.set_ylabel('y')

    # Plot spawn point and arrow for spawn orientation
    ax.plot(spawn_point.location.x, spawn_point.location.y, 'bo')
    theta_spawn = spawn_point.rotation.yaw / 180 * ca.pi
    ax.arrow(spawn_point.location.x, spawn_point.location.y, 0.5 * ca.cos(theta_spawn), 0.5 * ca.sin(theta_spawn),
             width=0.1)

    # Plot current state and goal state / goal orientation
    ax.plot(x0, y0, 'go')
    ax.plot(waypoints[i][0], waypoints[i][1], 'ro')
    waypoint_x = float(waypoints[i][0].full())
    waypoint_y = float(waypoints[i][1].full())
    ax.arrow(waypoint_x, waypoint_y, 0.5 * ca.cos(0), 0.5 * ca.sin(0), width=0.1)

    # Plot control input as arrow (acceleration and steering angle)
    ax.arrow(x0, y0, 0.5 * u[1] * ca.cos(theta0 + u[0]), 0.5 * u[1] * ca.sin(theta0 + u[0]), width=0.1, color='r')

    # Plot open-loop trajectory
    ax.plot(opti.debug.value(X)[0, :], opti.debug.value(X)[1, :], 'b-')

    # Plot closed-loop trajectory
    ax.plot([x[0] for x in closed_loop_data], [x[1] for x in closed_loop_data], 'g-')

    # Display cost and iteration number outside plot
    ax.text(0.5, 1.05, f"Final Cost: {sol.value(obj):.2f}", horizontalalignment='center',
            verticalalignment='center',
            transform=ax.transAxes)

    # Explicitly set the legend elements and labels, and display legend outside of plot
    legend_elements = [plt.Line2D([0], [0], color='b', marker='o', label='Spawn point'),
                       plt.Line2D([0], [0], color='g', marker='o', label='Current state'),
                       plt.Line2D([0], [0], color='r', marker='o', label='Goal state'),
                       plt.Line2D([0], [0], color='b', label='Open-loop trajectory'),
                       plt.Line2D([0], [0], color='g', label='Closed-loop trajectory')]
    ax.legend(handles=legend_elements, loc='center left', bbox_to_anchor=(1, 0.5))

    # Extend figure to fit legend, but make sure plot is still properly sized
    fig.tight_layout(rect=[0, 0, 1, 1])

    return fig


def generate_plot_inner(spawn_point, x0, y0, theta0, waypoints, i, X, opti, obj, closed_loop_data):
    # Plot open-loop trajectory
    fig, ax = plt.subplots(1, 1)
    ax.set_xlabel('x')
    ax.set_ylabel('y')

    # Plot spawn point and arrow for spawn orientation
    ax.plot(spawn_point.location.x, spawn_point.location.y, 'bo')
    theta_spawn = spawn_point.rotation.yaw / 180 * ca.pi
    ax.arrow(spawn_point.location.x, spawn_point.location.y, 0.5 * ca.cos(theta_spawn), 0.5 * ca.sin(theta_spawn),
             width=0.1)

    # Plot current state and goal state / goal orientation
    ax.plot(x0, y0, 'go')
    ax.plot(waypoints[i][0], waypoints[i][1], 'ro')
    waypoint_x = float(waypoints[i][0].full())
    waypoint_y = float(waypoints[i][1].full())
    ax.arrow(waypoint_x, waypoint_y, 0.5 * ca.cos(0), 0.5 * ca.sin(0), width=0.1)

    # Plot open-loop trajectory
    ax.plot(opti.debug.value(X)[0, :], opti.debug.value(X)[1, :], 'b-')

    # Plot closed-loop trajectory
    ax.plot([x[0] for x in closed_loop_data], [x[1] for x in closed_loop_data], 'g-')

    # Display cost and iteration number outside plot
    ax.text(0.5, 1.05, f"Final Cost: {opti.debug.value(obj):.2f}", horizontalalignment='center',
            verticalalignment='center',
            transform=ax.transAxes)

    # Explicitly set the legend elements and labels, and display legend outside of plot
    legend_elements = [plt.Line2D([0], [0], color='b', marker='o', label='Spawn point'),
                       plt.Line2D([0], [0], color='g', marker='o', label='Current state'),
                       plt.Line2D([0], [0], color='r', marker='o', label='Goal state'),
                       plt.Line2D([0], [0], color='b', label='Open-loop trajectory'),
                       plt.Line2D([0], [0], color='g', label='Closed-loop trajectory')]
    ax.legend(handles=legend_elements, loc='center left', bbox_to_anchor=(1, 0.5))

    # Extend figure to fit legend, but make sure plot is still properly sized
    fig.tight_layout(rect=[0, 0, 1, 1])

    return fig
