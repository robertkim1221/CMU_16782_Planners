import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

import sys

SPEEDUP = 10

def parse_mapfile(filename):
    with open(filename, 'r') as file:
        assert file.readline().strip() == 'N', "Expected 'N' in the first line"
        x_size_, y_size_ = map(int, file.readline().strip().split(','))

        assert file.readline().strip() == 'C', "Expected 'C' in the third line"
        collision_thresh = int(file.readline().strip())

        assert file.readline().strip() == 'R', "Expected 'R' in the fifth line"
        robotX, robotY = map(int, file.readline().strip().split(','))

        assert file.readline().strip() == 'T', "Expected 'T' in the seventh line"
        target_traj = []
        line = file.readline().strip()
        while line != 'M':
            x, y = map(float, line.split(','))
            target_traj.append({'x': x, 'y': y})
            line = file.readline().strip()

        costmap_ = []
        for line in file:
            row = list(map(float, line.strip().split(',')))
            costmap_.append(row)

        costmap_ = np.asarray(costmap_).T

    return x_size_, y_size_, collision_thresh, robotX, robotY, target_traj, costmap_


def parse_robot_trajectory_file(filename):
    robot_traj = []
    with open(filename, 'r') as file:
        for line in file:
            t, x, y = map(int, line.strip().split(','))
            robot_traj.append({'t': t, 'x': x, 'y': y})

    return robot_traj


def animate():
    fig, ax = plt.subplots()

    ax.imshow(costmap, cmap='jet')

    line1, = ax.plot([], [], lw=2, marker='o', color='b', label='robot')
    line2, = ax.plot([], [], lw=2, marker='o', color='r', label='target')

    def init():
        line1.set_data([], [])
        line2.set_data([], [])
        return line1, line2

    def update(frame):
        frame *= SPEEDUP
        line1.set_data([p['x'] for p in robot_trajectory[:frame + 1]], [p['y'] for p in robot_trajectory[:frame + 1]])

        t = robot_trajectory[frame + 1]['t']
        line2.set_data([p['x'] for p in target_trajectory[:t]], [p['y'] for p in target_trajectory[:t]])

        # plt.pause((robot_trajectory[frame+1]['t']-robot_trajectory[frame]['t'])/SPEEDUP)
        return line1, line2

    ani = FuncAnimation(fig, update, frames=(len(robot_trajectory) - 1) // SPEEDUP, init_func=init, blit=False,
                        interval=1)

    plt.legend()
    plt.show()


def plot_final():
    fig, ax = plt.subplots()

    ax.imshow(costmap, cmap='jet')
    ax.plot([p['x'] for p in robot_trajectory], [p['y'] for p in robot_trajectory], lw=2, marker='o', color='b',
            label='robot')
    ax.plot([p['x'] for p in target_trajectory], [p['y'] for p in target_trajectory], lw=2, marker='o', color='r',
            label='target')
    plt.legend()
    plt.show()


if __name__ == "__main__":

    animate_bool = True
    if len(sys.argv) < 2:
        print("Usage: python visualizer.py <map filename>")
        sys.exit(1)

    if len(sys.argv) < 3:
        print("Default: animating")
    else:
        if sys.argv[2] == 'True' or sys.argv[2] == 'true':
            animate_bool = True
        else:
            animate_bool = False

    x_size, y_size, collision_threshold, robotX, robotY, target_trajectory, costmap = parse_mapfile(sys.argv[1])

    robot_trajectory = parse_robot_trajectory_file('../output/robot_trajectory.txt')
    target_trajectory = target_trajectory[:robot_trajectory[-1]['t']]
    if animate_bool:
        animate()
    else:
        plot_final()
