# navmesh_explanations

## Objective

Given the shortest path A between a start and goal location on a navigation mesh, this package computes an explanation for why the shortest path is A instead of B (provided by the user).

## Citation

If you use this in your research, please cite:

> Martim Brandao, Amanda Coles, Daniele Magazzeni, "**Explaining path plan optimality: fast explanation methods for navigation meshes using full and incremental inverse optimization**", *ICAPS 2021*.

## Requirements

This package uses python3 (for cvxpy).

```
sudo apt install python3-pip
pip3 install rospkg networkx cvxpy cvxopt similaritymeasures tabulate matplotlib
```

It also requires the ROS package [recast_ros](https://github.com/ori-drs/recast_ros) for interfacing with navigation meshes.

## Usage

- Launch recast_ros with your preferred map
- Launch this package's interface.launch
- Run this package's test_recast_explanations.py
- Use right-clicking on the Rviz window to indicate your start point, goal point, and expected-waypoint
- Select in Rviz which kind of explanation you would like to see by enabling the appropriate topic (topic names should be quite self-explanatory)
