#!/usr/bin/env python3
import rospy
from minimum_snap_trajectory import MinimumSnapTrajectory
from math import pi, cos, sin, atan2, sqrt
import matplotlib.pyplot as plt
from quadrotor_sim.srv import TrajectoryVisualizer, FlatOutputs
import sys
from std_srvs.srv import Empty
import signal
import os

class TrajectoryPlanner:
    def __init__(self, keyframes, timePoints, psi_tangent_to_traj):
        # initialize the node
        rospy.init_node("trajectory_planner_node")
        # seperating x, y, z, psi component of the keyframes
        x = list(map(lambda keyframe: keyframe[0], keyframes))
        y = list(map(lambda keyframe: keyframe[1], keyframes))
        z = list(map(lambda keyframe: keyframe[2], keyframes))

        self.dt = 0.01
        # planning the trajectory
        self.x, self.xdot, self.xddot, self.xdddot, self.t = MinimumSnapTrajectory(x, timePoints, self.dt, degree=6, c=4)
        self.y, self.ydot, self.yddot, self.ydddot, self.t = MinimumSnapTrajectory(y, timePoints, self.dt, degree=6, c=4)
        self.z, self.zdot, self.zddot, self.zdddot, self.t = MinimumSnapTrajectory(z, timePoints, self.dt, degree=6, c=4)

        # the user can plan PSI or ask it to be tangent to the trajectory at any given keyframe
        if psi_tangent_to_traj:
            psi = list()
            for T in timePoints:
                index = int(T / self.dt) if T == 0 else (int(T / self.dt) - 1)
                psi.append(atan2(self.ydot[index], self.xdot[index]))
                keyframes[len(psi) - 1][3] = psi[-1]
        else:
            psi = list(map(lambda keyframe: keyframe[3], keyframes))

        self.psi, self.psidot, self.psiddot, self.psidddot, self.t = MinimumSnapTrajectory(psi, timePoints, self.dt, degree=6, c=4)

        self.keyframes = keyframes
        # waiting for service to visualize the trajectory inside gazebo
        rospy.wait_for_service("/trajectory/visualizer")
        self.trajectory_visualizer = rospy.ServiceProxy("/trajectory/visualizer", TrajectoryVisualizer)
        self.clear_marker_client = rospy.ServiceProxy("/clear_markers", Empty)
        # ros service to set the flatoutputs to the controller
        self.flatoutputs_service = rospy.ServiceProxy("/set/flatoutputs", FlatOutputs)

    # method to track the trajectory
    def track(self):
        controller_rate = rospy.Rate(1 / self.dt)
        i = 0
        # track the trajectory
        while not rospy.is_shutdown() and i < len(self.x):
            flatoutput = FlatOutputs()._request_class()
            flatoutput.rT = [self.x[i], self.y[i], self.z[i]]
            flatoutput.rTdot = [self.xdot[i], self.ydot[i], self.zdot[i]]
            flatoutput.rTddot = [self.xddot[i], self.yddot[i], self.zddot[i]]
            flatoutput.adot = [self.xdddot[i], self.ydddot[i], self.zdddot[i]]
            flatoutput.psi_T = self.psi[i]
            flatoutput.psidot_T = self.psidot[i]
            self.flatoutputs_service.call(flatoutput)
            controller_rate.sleep()
            i += 1

        rospy.Rate(1).sleep()
        self.clear_marker_client.call()
        self.clear_marker_client.wait_for_service()
        # return to launch
        self.return_to_launch()

    def visualize_trajectory(self):
        self.trajectory_visualizer(self.x, self.y, self.z, len(self.x))

    # a method to plot the planned trajectories
    def plot_trajectory(self):
        # plotting the result
        plt.figure()
        plt.title("X")
        plt.plot(self.t , self.x, color="red", linewidth=3, label="position")
        plt.plot(self.t , self.xdot, color="green", linewidth=3, label="velocity")
        plt.plot(self.t , self.xddot, color="blue", linewidth=3, label="acceleration")
        plt.legend()
        plt.grid()

        plt.figure()
        plt.title("Y")
        plt.plot(self.t , self.y, color="red", linewidth=3, label="position")
        plt.plot(self.t , self.ydot, color="green", linewidth=3, label="velocity")
        plt.plot(self.t , self.yddot, color="blue", linewidth=3, label="acceleration")
        plt.legend()
        plt.grid()

        plt.figure()
        plt.title("Z")
        plt.plot(self.t , self.z, color="red", linewidth=3, label="position")
        plt.plot(self.t , self.zdot, color="green", linewidth=3, label="velocity")
        plt.plot(self.t , self.zddot, color="blue", linewidth=3, label="acceleration")
        plt.legend()
        plt.grid()

        plt.figure()
        plt.title("PSI")
        plt.plot(self.t , self.psi, color="red", linewidth=3, label="position")
        plt.plot(self.t , self.psidot, color="green", linewidth=3, label="velocity")
        plt.plot(self.t , self.psiddot, color="blue", linewidth=3, label="acceleration")
        plt.legend()
        plt.grid()

        plt.figure()
        plt.title("Trjactory")
        plt.plot(self.x, self.y, color="red", linewidth=3, label="position")
        plt.legend()
        plt.grid()
        for keyframe in self.keyframes:
            x, y, _, psi = keyframe
            # Calculate the arrow directions based on psi
            dx, dy = cos(psi), sin(psi)
            plt.quiver(x, y, dx, dy, color="blue", scale=20, width=0.005)

        plt.show()
    # method to return the quadrotor to origin
    # we will call this method after tracking is done
    def return_to_launch(self):
        flatoutput = FlatOutputs()._request_class()
        flatoutput.rT = [0.0, 0.0, 0.0]
        flatoutput.rTdot = [0.0, 0.0, 0.0]
        flatoutput.rTddot = [0.0, 0.0, 0.0]
        flatoutput.adot = [0.0, 0.0, 0.0]
        flatoutput.psi_T = 0.0
        flatoutput.psidot_T = 0.0
        self.flatoutputs_service.call(flatoutput)


if __name__ == "__main__":
    trajectory_number = 1
    if len(sys.argv) > 1:
        trajectory_number = int(sys.argv[1])

    if trajectory_number == 1: # trajectory 1
        keys = [
            [0, 0, 0, 0],
            [1, 0, 2, 0],
            [1, 2, 2, 0],
            [0, 2, 0, 0],
            [1, 2, 2, 0],
            [1, 0, 2, 0],
            [0, 0, 0, 0]
        ]
        tPoints = [0, 2, 4, 6, 8, 10, 12]

    elif trajectory_number == 2: # trajectory 2
        keys = [
            [0, 0, 0, 0],
            [2, -2, 1, 0],
            [3, 5, 1, 0],
            [7, -1, 1, 0],
            [4, -3, 1, 0],
            [2, -1, 1, 0],
            [0, 0, 0, 0]
        ]
        tPoints = [0, 2, 4, 6, 8, 10, 12]
    
    elif trajectory_number == 3: # trajectory 3
        keys = [
            [0, 0, 0, 0],
            [3, 3, 2, 0], 
            [4, -4, 2, 0], 
            [6, -7, 2, 0],
            [0, 0, 0, 0]
        ]
        tPoints = [0, 2, 4, 6, 10]

    elif trajectory_number == 4: # trajectory 4
        keys = [
            [0, 0, 0, 0],
            [5, 0, 2, 0],
            [10, 0, 2, 0],
            [7.5, 5*sqrt(3)/2, 2, 0],
            [5, 5*sqrt(3), 2, 0],
            [2.5, 5*sqrt(3)/2, 2, 0],
            [0, 0, 0, 0]
        ]
        tPoints = [0, 2, 4, 6, 8, 10, 12]

    elif trajectory_number == 5: # trajectory 5
        keys = [
            [0, 0, 0, 0],
            [3, 2, 2, 0],
            [4, 5, 2, 0],
            [2, 7, 2, 0],
            [8, 3, 2, 0],
            [5, -4, 2, 0],
            [0, 0, 0, 0]
        ]
        tPoints = [0, 1.5, 2.5, 3.5, 5, 7, 10]

    elif trajectory_number == 6:
        keys = [
            [0, 0, 0, 0],
            [3, -0.8, 2, 0],
            [6, -0.15, 2, 0], 
            [9, 2, 2, 0], 
            [10, 1, 2, 0],
            [9, 0, 2, 0],
            [6, -0.15, 2, 0],
            [3.8, -0.2, 2, 0],
            [4, 0, 2, 0]
        ]
        tPoints = [0, 1.5, 2, 3, 3.5, 4, 4.5, 7, 7.5]

    trajectory_planner_node = TrajectoryPlanner(keys, tPoints, psi_tangent_to_traj=True)
    trajectory_planner_node.visualize_trajectory()
    trajectory_planner_node.track()
    # trajectory_planner_node.plot_trajectory()

    # terminating the launch
    pid = os.getpid()
    os.kill(pid, signal.SIGINT)


