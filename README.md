# Quadrotor_sim

Quadrotor_sim is a ROS Noetic quadrotor simulation which implements a quadrotor trajectory planner + a nonlinear controller.

## Installation

To install this repository, first make sure you clone this repository in `catkin_ws/src` and build your workspace by running this command in `catkin_ws`:

```bash
catkin_make
```

## Running

Start the Gazebo simulation with:

```bash
roslaunch quadrotor_sim gazebo.launch
```

This command will spawn the quadrotor and start the nonlinear controller.

## Starting Trajectories

This project currently has 6 premade trajectories. To run these trajectories, replace `<num>` with the desired trajectory number and run this command in a separate terminal:

```bash
roslaunch quadrotor_sim trajectory_<num>.launch
```

## YouTube Video

Here is a [YouTube Video](https://www.youtube.com/watch?v=H3kuU0CeNwU) of the project.

## Ref

This simulation is the implementation of the following paper:

```
D. Mellinger and V. Kumar, "Minimum snap trajectory generation and control for quadrotors," 2011 IEEE International Conference on Robotics and Automation, Shanghai, China, 2011, pp. 2520-2525, doi: 10.1109/ICRA.2011.5980409.
```

## Upcoming Changes

In the following version of this project, I aim to:

1. Add Extended Kalman filter for state estimation.
2. Add a project report PDF file.
