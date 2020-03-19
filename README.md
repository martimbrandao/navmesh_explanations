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
- python gurobipy

TODO:
- [x] navmesh topic graph
- [x] python ros getgraph
- [x] python convert to networkx
- [x] networkx get shortest path, compare
- [x] networkx get k shortest paths
- [ ] ros show k shortest paths
- [ ] gurobipy solve inverse shortest path (with A only or k-shortest-paths also as constraints)
- [ ] reformulate so that paths not constraints but costs?

