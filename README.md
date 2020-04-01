# navmesh_explanations

## Objective

Basic:
- navmesh set start s goal g, get path A (s-g)
- navmesh set desired waypoint w, get path B (s-w-g)
- get graph G, and A/B as sequences of nodes
- S_0 = A
- min ||c_i - c_i'||  s.t.  sum_{k in S_j} c_k >= sum_{k in B} c_k

Could potentially add:
- get N random walks S_j (s-g) for j = 1,...,N
- get N independent rrt paths S_j (s-g) for j = 1,...,N
- get k-shortest-paths of G into S_j (s-g) for j = 1,...,k

Modeling:
- python networkX (shortest_simple_paths, betweenness, closeness, etc.)
- python gurobipy cvxpy

TODO:
- [x] navmesh topic graph
- [x] python ros getgraph
- [x] python convert to networkx
- [x] networkx get shortest path, compare
- [x] networkx get k shortest paths
- [x] ros show k shortest paths
- [x] cvxpy solve inverse shortest path (with k-shortest-paths as constraints)
- [x] cvxpy optPolyLabels from optPolyCosts (e.g. for each node choose areatype except current that has closest cost to that in x)
- [x] rviz show cvxpy solutions
- [x] incrementally adding new k-shortest paths to constraints (until max iter or until desired is present in k-shortest)
- [ ] compare k-shortest paths to: 1) random walks (RRTs); 2) diverse short paths (Voss 2015);
- [x] compare k-shortest paths to: 3) diverse short paths using weights 4) edge-disjoint paths
- [x] compare norm2 to norm1 (norm1 more stable and finds solutions more often)
- [x] reformulate so that alternative paths are not constraints but costs

MAYBE:
- [ ] cvxpy optPolyLabels with path nodes only as variables (might have to iterate)

## Requirements

This package uses python3 (for cvxpy).

```
sudo apt install python3-pip
pip3 install rospkg networkx cvxpy cvxopt
```

