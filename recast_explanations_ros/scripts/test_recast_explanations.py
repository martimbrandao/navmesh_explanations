#!/usr/bin/env python3

import networkx as nx
import numpy as np
import cvxpy as cp
import similaritymeasures
import matplotlib
import matplotlib.pyplot as plt
import time
import random
import pdb
import rospy
from dynamic_reconfigure.server import Server
from recast_explanations_ros.cfg import ExplanationsConfig
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from recast_ros.srv import RecastProjectSrv, RecastProjectSrvRequest
from recast_ros.msg import RecastGraph, RecastGraphNode
from itertools import islice, product
from tabulate import tabulate
from queue import PriorityQueue


### functions

def getProj(point):
  try:
    srv = rospy.ServiceProxy('/recast_node/project_point', RecastProjectSrv)
    query = RecastProjectSrvRequest()
    query.point = point
    proj = srv(query)
  except rospy.ServiceException:
    rospy.logerr('could not project point')
    exit()
  return proj


def getKey(pt):
  return (pt.x, pt.y, pt.z)


def addNode(rosnode, areaCosts, nodeDict):
  key = getKey(rosnode.point)
  # NOTE: A previously added point can arrive here again with a different area type sometimes, for some reason. So I just stick to the first node data I get...
  if key in nodeDict:
    return key, nodeDict[key]
  data = {}
  data["id"] = nodeDict[key]["id"] if key in nodeDict else len(nodeDict)
  data["point"] = rosnode.point
  data["area"] = rosnode.area_type.data
  data["cost"] = areaCosts[rosnode.area_type.data]
  data["portal"] = False
  #if key in nodeDict and nodeDict[key] != data:
  #  return key, nodeDict[key]
  nodeDict[key] = data
  return key, data


def addPortal(rospoint, nodeDict):
  key = getKey(rospoint)
  if key in nodeDict:
    return key, nodeDict[key]
  data = {}
  data["id"] = nodeDict[key]["id"] if key in nodeDict else len(nodeDict)
  data["point"] = rospoint
  data["area"] = -1
  data["cost"] = -1
  data["portal"] = True
  #if key in nodeDict and nodeDict[key] != data:
  #  return key, nodeDict[key]
  nodeDict[key] = data
  return key, data


def copyDict(dict1, dict2):
  for k in dict1:
    dict2[k] = dict1[k]


def dist(p1, p2):
  return np.linalg.norm(np.array([p2.x, p2.y, p2.z]) - np.array([p1.x, p1.y, p1.z]))


def distanceMatrix(graph):
  D = {}
  for node1 in graph:
    D[node1] = {}
    for node2 in graph:
      D[node1][node2] = dist(graph.nodes[node1]["point"], graph.nodes[node2]["point"])
  return D


def getCost(graph, path):
  cost = 0
  for i in range(len(path)-1):
    cost += graph[path[i]][path[i+1]]["weight"]
  return cost


def newPoint(point, height):
  newpoint = Point()
  newpoint.x = point.x
  newpoint.y = point.y
  newpoint.z = point.z + height
  return newpoint


def newMarker(id, type, scale, color):
  marker = Marker()
  marker.header.frame_id = 'map'
  marker.header.stamp = rospy.Time.now()
  marker.id = id
  marker.action = Marker.ADD
  marker.scale.x = scale
  if type != Marker.LINE_LIST:
    marker.scale.y = scale
    marker.scale.z = scale
  marker.type = type
  marker.color.r = color[0]
  marker.color.g = color[1]
  marker.color.b = color[2]
  marker.color.a = color[3]
  marker.pose.orientation.w = 1
  return marker


def pathToMarker(graph, path, id, color, height):
  marker = newMarker(id, Marker.LINE_LIST, 0.1, color)
  for i in range(len(path)-1):
    marker.points.append(newPoint(graph.nodes[path[i  ]]["point"], height))
    marker.points.append(newPoint(graph.nodes[path[i+1]]["point"], height))
  return marker


def pathsToMarkerArray(graph, paths, height):
  colors = ['#e6194b', '#3cb44b', '#ffe119', '#4363d8', '#f58231', '#911eb4', '#46f0f0', '#f032e6', '#bcf60c', '#fabebe', '#008080', '#e6beff', '#9a6324', '#fffac8', '#800000', '#aaffc3', '#808000', '#ffd8b1', '#000075', '#808080', '#ffffff', '#000000']
  markers = MarkerArray()
  for i in range(len(paths)):
    color = [int(colors[i % len(colors)][j:j + 2], 16) / 255. for j in (1, 3, 5)] + [1]
    #pdb.set_trace()
    m = pathToMarker(graph, paths[i], i, color, height)
    markers.markers.append(m)
  return markers


def graphToMarkerArray(graph, height):
  # edges
  graphmarker1 = newMarker(0, Marker.LINE_LIST, 0.05, [0,0,1,1])
  graphmarker2 = newMarker(1, Marker.LINE_LIST, 0.05, [1,0,1,1])
  for edge in list(graph.edges):
    if graph[edge[0]][edge[1]]["area"] == 1:
      graphmarker1.points.append(newPoint(graph.nodes[edge[0]]["point"], height))
      graphmarker1.points.append(newPoint(graph.nodes[edge[1]]["point"], height))
    elif graph[edge[0]][edge[1]]["area"] == 2:
      graphmarker2.points.append(newPoint(graph.nodes[edge[0]]["point"], height))
      graphmarker2.points.append(newPoint(graph.nodes[edge[1]]["point"], height))
  # nodes
  nodemarker1 = newMarker(2, Marker.SPHERE_LIST, 0.2, [0,0,1,1])
  nodemarker2 = newMarker(3, Marker.SPHERE_LIST, 0.2, [1,0,1,1])
  nodemarker3 = newMarker(4, Marker.SPHERE_LIST, 0.1, [0,1,0,1])
  for node in list(graph.nodes):
    if graph.nodes[node]["area"] == 1:
      nodemarker1.points.append(newPoint(graph.nodes[node]["point"], height))
    elif graph.nodes[node]["area"] == 2:
      nodemarker2.points.append(newPoint(graph.nodes[node]["point"], height))
    elif graph.nodes[node]["area"] == -1:
      nodemarker3.points.append(newPoint(graph.nodes[node]["point"], height))
  # return
  graphmarkers = MarkerArray()
  if len(graphmarker1.points) > 0:
    graphmarkers.markers.append(graphmarker1)
  if len(graphmarker2.points) > 0:
    graphmarkers.markers.append(graphmarker2)
  if len(nodemarker1.points) > 0:
    graphmarkers.markers.append(nodemarker1)
  if len(nodemarker2.points) > 0:
    graphmarkers.markers.append(nodemarker2)
  #if len(nodemarker3.points) > 0:
  #  graphmarkers.markers.append(nodemarker3)
  return graphmarkers


def graphToMarkerArrayByArea(graph, height, area_types):
  colors = ['#e6194b', '#3cb44b', '#ffe119', '#4363d8', '#f58231', '#911eb4', '#46f0f0', '#f032e6', '#bcf60c', '#fabebe', '#008080', '#e6beff', '#9a6324', '#fffac8', '#800000', '#aaffc3', '#808000', '#ffd8b1', '#000075', '#808080', '#ffffff', '#000000']
  # init
  area2edgemarkerIdx = {}
  area2nodemarkerIdx = {}
  edgemarkers = []
  nodemarkers = []
  id = 0
  for i in range(len(area_types)):
    color = [int(colors[i % len(colors)][j:j + 2], 16) / 255. for j in (1, 3, 5)] + [1]
    area2edgemarkerIdx[ area_types[i] ] = len(edgemarkers)
    edgemarkers.append( newMarker(id, Marker.LINE_LIST, 0.05, color) )
    id += 1
    area2nodemarkerIdx[ area_types[i] ] = len(nodemarkers)
    nodemarkers.append( newMarker(id, Marker.SPHERE_LIST, 0.2, color) )
    id += 1
  # edges
  for edge in list(graph.edges):
    idx = area2edgemarkerIdx[ graph[edge[0]][edge[1]]["area"] ]
    edgemarkers[idx].points.append(newPoint(graph.nodes[edge[0]]["point"], height))
    edgemarkers[idx].points.append(newPoint(graph.nodes[edge[1]]["point"], height))
  # nodes
  for node in list(G.nodes):
    idx = area2nodemarkerIdx[ graph.nodes[node]["area"] ]
    nodemarkers[idx].points.append(newPoint(graph.nodes[node]["point"], height))
  # return
  graphmarkers = MarkerArray()
  for em in edgemarkers:
    if len(em.points) > 0:
      graphmarkers.markers.append(em)
  for nm in nodemarkers:
    if len(nm.points) > 0:
      graphmarkers.markers.append(nm)
  return graphmarkers

def graphToMarkerArrayByCost(graph, height):
  nbins = 20
  colors = plt.get_cmap("rainbow", nbins) # cool, rainbow
  # discretize costs
  costs = []
  for edge in list(graph.edges):
    costs.append( max(graph.nodes[edge[0]]["cost"], graph.nodes[edge[1]]["cost"]) )
  costs = np.array(costs)
  bins = np.linspace(np.min(costs), np.max(costs), nbins-1)
  # init
  edgemarkers = []
  for i in range(nbins):
    edgemarkers.append( newMarker(i, Marker.LINE_LIST, 0.05, colors(i)) )
  nodemarkers = []
  for i in range(nbins):
    nodemarkers.append( newMarker(len(edgemarkers)+i, Marker.SPHERE_LIST, 0.2, colors(i)) )
  nodemarkerportals = newMarker(len(edgemarkers)+len(nodemarkers), Marker.SPHERE_LIST, 0.1, [0,1,0,1])
  # edges
  for edge in list(graph.edges):
    cost = max(graph.nodes[edge[0]]["cost"], graph.nodes[edge[1]]["cost"])
    idx = np.digitize(cost, bins)
    edgemarkers[idx].points.append(newPoint(graph.nodes[edge[0]]["point"], height))
    edgemarkers[idx].points.append(newPoint(graph.nodes[edge[1]]["point"], height))
  # nodes
  for node in list(graph.nodes):
    if graph.nodes[node]["area"] == -1:
      nodemarkerportals.points.append(newPoint(graph.nodes[node]["point"], height))
    else:
      cost = graph.nodes[node]["cost"]
      idx = np.digitize(cost, bins)
      nodemarkers[idx].points.append(newPoint(graph.nodes[node]["point"], height))
  # return
  graphmarkers = MarkerArray()
  for em in edgemarkers:
    if len(em.points) > 0:
      graphmarkers.markers.append(em)
  for nm in nodemarkers:
    if len(nm.points) > 0:
      graphmarkers.markers.append(nm)
  #graphmarkers.markers.append(nodemarkerportals)
  return graphmarkers

def getGraphChangesForVis(graph1, graph2):
  gchanges = graph1.copy()
  for node in graph1:
    if graph1.nodes[node]["area"] == graph2.nodes[node]["area"]:
      gchanges.nodes[node]["area"] = 1
    else:
      gchanges.nodes[node]["area"] = 2
  changes = 0
  for edge in list(graph1.edges):
    if graph1[edge[0]][edge[1]]["area"] == graph2[edge[0]][edge[1]]["area"]:
      gchanges[edge[0]][edge[1]]["area"] = 1
    else:
      gchanges[edge[0]][edge[1]]["area"] = 2
      changes += 1
  rospy.loginfo("edge area changes = %d" % changes)
  return gchanges


### search

def applyChangesToGraph(originalgraph, changes, areaCosts):
  graph = originalgraph.copy()
  for node in changes:
    area = changes[node]
    cost = areaCosts[area]
    graph.nodes[node]["area"] = area
    graph.nodes[node]["cost"] = cost
    for nei in graph.adj[node]:
      graph[node][nei]["area"] = area
      graph[node][nei]["cost"] = cost
      graph[node][nei]["weight"] = cost * dist(graph.nodes[node]["point"], graph.nodes[nei]["point"])
  return graph


def astarGetSuccessors(originalgraph, changes, pathToDecreaseCost, pathToIncreaseCost, areaCosts, allowedAreaTypes):
  graph = applyChangesToGraph(originalgraph, changes, areaCosts)
  costs = [areaCosts[area] for area in allowedAreaTypes]
  mincost = min(costs)
  maxcost = max(costs)
  successors = []
  for i in range(len(pathToDecreaseCost)-1):
    edge = [pathToDecreaseCost[i], pathToDecreaseCost[i+1]]
    cost = graph[edge[0]][edge[1]]["cost"]
    node = edge[0] if graph.nodes[edge[1]]["portal"] else edge[1]
    if node not in changes and cost > mincost:
      for newarea in allowedAreaTypes:
        newcost = areaCosts[newarea]
        if newcost < cost:
          newchanges = changes.copy()
          newchanges[node] = newarea
          changeCost = 1
          successors.append([newchanges, changeCost])
  for i in range(len(pathToIncreaseCost)-1):
    edge = [pathToIncreaseCost[i], pathToIncreaseCost[i+1]]
    cost = graph[edge[0]][edge[1]]["cost"]
    node = edge[0] if graph.nodes[edge[1]]["portal"] else edge[1]
    if node not in changes and cost < maxcost:
      for newarea in allowedAreaTypes:
        newcost = areaCosts[newarea]
        if newcost > cost:
          newchanges = changes.copy()
          newchanges[node] = newarea
          changeCost = 10  # not-on-path nodes 10 times less preferred (similar to optimization-based methods)
          successors.append([newchanges, changeCost])
  return successors


def astarHeuristic(originalgraph, changes, desiredPath):
  return 0


def astarPolyLabels(graph, desiredPath, areaCosts, allowedAreaTypes, verbose):
  # Chakraborti et al, "Plan Explanations as Model Reconciliation: Moving Beyond Explanation as Soliloquy", 2017
  # https://github.com/TathagataChakraborti/mmp/
  # note: using heuristic successors for speed, bypassing original implementation to speed up results
  #       i.e. using nx.shortest_path instead of fast-downward + VAL
  startState            = {}
  fringe                = PriorityQueue()
  closed                = set()
  numberOfNodesExpanded = 0
  numberOfNodesAdded    = 0
  fringe.put((0, 0, [startState, 0]))
  while not fringe.empty():
    node = fringe.get()[2]
    newgraph = applyChangesToGraph(graph, node[0], areaCosts)
    newpath = nx.shortest_path(newgraph, source=desiredPath[0], target=desiredPath[-1], weight="weight")
    if pathDistance(desiredPath, newpath) == 0:
      return [], newgraph
    if frozenset(node[0]) not in closed:
      closed.add(frozenset(node[0]))
      successor_list = astarGetSuccessors(graph, node[0], desiredPath, newpath, areaCosts, allowedAreaTypes)
      numberOfNodesExpanded  += 1
      if verbose:
        rospy.loginfo("%d models expanded... current depth: %d" % (numberOfNodesExpanded, node[1]))
      while successor_list:
        candidate_node      = successor_list.pop()
        numberOfNodesAdded += 1
        new_node            = [candidate_node[0], node[1] + candidate_node[1]]
        fringe.put((astarHeuristic(graph, candidate_node, desiredPath) + new_node[1], numberOfNodesAdded, new_node))
  if verbose:
    rospy.loginfo("astarPolyLabels failed to find solution")
  return None


### optimization

def findShortestPathLP(graph, start, goal):
  # variable: x_ij                                              # vector of indicator variables saying if edge ij is part of the path
  # cost: sum_ij w_ij * x_ij
  # constraints: sum_j x_ij - sum_j x_ji = 1 (for i = start)    # outcoming edges - incoming edges = 1
  #              sum_j x_ij - sum_j x_ji =-1 (for i = goal)     # outcoming edges - incoming edges =-1
  #              sum_j x_ij - sum_j x_ji = 0 (other i)          # outcoming edges - incoming edges = 0

  edge2index = {}
  edges = []
  weights = []
  for (i,j) in graph.edges:
    edge2index[i,j] = len(edges)
    edges.append([i,j])
    weights.append(graph[i][j]["weight"])
    edge2index[j,i] = len(edges)
    edges.append([j,i])
    weights.append(graph[j][i]["weight"])

  A = []
  b = []
  for n in graph.nodes:
    # sum_j x_ij - sum_j x_ji
    line = [0]*len(edges)
    for i,j in edge2index.keys():
      if i == n:
        line[edge2index[i,j]] += 1
    for j,i in edge2index.keys():
      if i == n:
        line[edge2index[j,i]] -= 1
    A.append(line)
    # = 1/-1/0
    if n == start:
      b.append(1)
    elif n == goal:
      b.append(-1)
    else:
      b.append(0)
  #for n in waypoints: # note: this would just add a "flying" edge with no connections to the shortest path...
  #  # sum_j x_ij = 1
  #  line = [0]*len(edges)
  #  for i,j in edge2index.keys():
  #    if i == n:
  #      line[edge2index[i,j]] += 1
  #  A.append(line)
  #  b.append(1)
  A = np.array(A)
  b = np.array(b)

  # solve with cvxpy
  x = cp.Variable(len(edges))
  cost = cp.sum(cp.multiply(x, weights))
  prob = cp.Problem(cp.Minimize(cost), [x >= 0, x <= 1, A @ x == b])
  value = prob.solve()
  if value == float('inf'):
    rospy.loginfo("... shortest path LP failed")
    return []

  # recover path
  path = [start]
  while path[-1] != goal:
    for i in range(len(edges)):
      if abs(x.value[i] - 1) < 1e-3 and edges[i][0] == path[-1]:
        path.append(edges[i][1])
        break
  return path


def optInvMILP(graph, desiredPath, areaCosts, allowedAreaTypes, exact=True, verbose=True):

  # variables:
  #   x_j:      indicator variable, whether edge j is part of the shortest path
  #   A_ij:     node-arc incidence matrix (rows are nodes, columns are edges) = 1 if j leaves i, -1 if j enters i, 0 otherwise
  #   b_i:      difference between entering and leaving edges = 1 if i start, -1 if i target, 0 otherwise
  #   pi_i:     dual variable
  #   lambda_j: dual variable
  #   c_j:      edge cost = dist_j * ac_0 * l_0 + dist_j * ac_1 * l_1 + ... = sum_(k in areas) dist_j * ac_k * l_ik
  #   l_ik:     node area one-hot encoding

  # problems:
  #   SP:  min  c.x,
  #        s.t. Ax=b, x>=0
  #   ISP: min  |l-l'|,
  #        s.t. sum_i a_ij * pi_i = sum_(k in areas) dist_j * ac_k * l_ik,              for all j in desired path
  #             sum_i a_ij * pi_i + lambda_j = sum_(k in areas) dist_j * ac_k * l_ik,   for all j not in desired path
  #             sum_k l_ik = 1                                                          for all i
  #             lambda >= 0,                                                            for all j not in desired path.

  cost_type = "label_changes"   # "weights" or "label_changes"

  # auxiliary variables
  edge2index = {}
  edges = []
  edge2varnodeindex = {}
  varnodes = []
  weights = []
  for (i,j) in graph.edges:
    edge2index[i,j] = len(edges)
    edges.append([i,j])
    edge2index[j,i] = len(edges)
    edges.append([j,i])
    if not graph.nodes[i]["portal"]:
      vn = i
    else:
      vn = j
    if vn in varnodes:
      idx = varnodes.index(vn)
    else:
      idx = len(varnodes)
      varnodes.append(vn)
    edge2varnodeindex[i,j] = idx
    edge2varnodeindex[j,i] = idx
    weights.append(graph[i][j]["weight"])
  node2index = {}
  nodes = []
  for n in graph.nodes:
    node2index[n] = len(nodes)
    nodes.append(n)
    if n == desiredPath[0]:
      s = node2index[n]
    if n == desiredPath[-1]:
      t = node2index[n]
  weights = np.array(weights)

  # l_original
  l_original = np.zeros(len(varnodes) * len(allowedAreaTypes))
  for (i,j) in graph.edges:
    idx = edge2varnodeindex[i,j]
    node = varnodes[idx]
    for k in range(len(allowedAreaTypes)):
      if allowedAreaTypes[k] == graph.nodes[node]["area"]:
        l_original[len(allowedAreaTypes) * idx + k] = 1
      else:
        l_original[len(allowedAreaTypes) * idx + k] = 0

  # Ax = b
  A = np.zeros([len(nodes), len(edges)])
  b = np.zeros(len(nodes))
  for i in range(len(nodes)):
    for nei in graph.adj[nodes[i]]:
      j = edge2index[nodes[i], nei]
      A[i,j] = 1
      j = edge2index[nei, nodes[i]]
      A[i,j] =-1
    if i == s:
      b[i] = 1
    if i == t:
      b[i] =-1

  # optimal x
  path = nx.shortest_path(graph, source=desiredPath[0], target=desiredPath[-1], weight="weight")
  xstar = np.zeros(len(edges))
  for p in range(len(path)-1):
    j = edge2index[path[p], path[p+1]]
    xstar[j] = 1

  # desired x
  xzero = np.zeros(len(edges))
  for p in range(len(desiredPath)-1):
    j = edge2index[desiredPath[p], desiredPath[p+1]]
    xzero[j] = 1

  # weights for L1norm (not-on-path-penalty)
  w = np.array([1]*len(l_original))
  for i in range(len(varnodes)):
    if varnodes[i] not in desiredPath[1:-1]:
      for k in range(len(allowedAreaTypes)):
        w[len(allowedAreaTypes) * i + k] *= 10

  # inverse optimization problem
  l_ = cp.Variable(len(l_original), boolean=True)
  pi_ = cp.Variable(len(nodes))
  lambda_ = cp.Variable(len(edges))
  # cost
  if cost_type == "weights":
    cost = 0
    j = 0
    for edge in graph.edges:
      i = edge2varnodeindex[edge[0], edge[1]]
      # edge's new cost d_j = sum_(k in areas) dist_j * ac_k * l_ik
      d_j = 0
      dist_j = dist(graph.nodes[edge[0]]["point"], graph.nodes[edge[1]]["point"])
      for k in range(len(allowedAreaTypes)):
        ac_k = areaCosts[allowedAreaTypes[k]]
        d_j += dist_j * ac_k * l_[len(allowedAreaTypes) * i + k]
      cost += cp.abs(d_j - weights[j])
      j += 1
  else:
    cost = cp.norm1(cp.multiply(l_ - l_original, w))  # cost = cp.norm1(l_ - l_original)
  # constraints
  constraints = []
  for j in range(len(edges)):
    edge = edges[j]
    i = edge2varnodeindex[edge[0], edge[1]]
    # edge's new cost d_j = sum_(k in areas) dist_j * ac_k * l_ik
    d_j = 0
    dist_j = dist(graph.nodes[edge[0]]["point"], graph.nodes[edge[1]]["point"])
    for k in range(len(allowedAreaTypes)):
      ac_k = areaCosts[allowedAreaTypes[k]]
      d_j += dist_j * ac_k * l_[len(allowedAreaTypes) * i + k]
    if xzero[j] == 1:
      # sum_i a_ij * pi_i = d_j,              for all j in desired path
      constraints.append( cp.sum(cp.multiply(A[:,j], pi_)) == d_j )
    else:
      # sum_i a_ij * pi_i + lambda_j = d_j,   for all j not in desired path
      constraints.append( cp.sum(cp.multiply(A[:,j], pi_)) + lambda_[j] == d_j )
  # sum_k l_ik = 1, for all i
  for i in range(len(varnodes)):
    idx = len(allowedAreaTypes) * i
    constraints.append( cp.sum(l_[idx:idx+len(allowedAreaTypes)]) == 1 )
  # lambda >= 0, for all j not in desired path.
  for j in range(len(edges)):
    if xzero[j] == 0:
      constraints.append( lambda_[j] >= 0 )

  # solve with cvxpy
  prob = cp.Problem(cp.Minimize(cost), constraints)
  if exact:
    value = prob.solve(solver=cp.MOSEK, mosek_params={"MSK_DPAR_MIO_MAX_TIME":-1, "MSK_DPAR_MIO_TOL_REL_GAP":0, "MSK_IPAR_MIO_CUT_CLIQUE":0, "MSK_IPAR_MIO_CUT_CMIR":0, "MSK_IPAR_MIO_CUT_GMI":0, "MSK_IPAR_MIO_CUT_SELECTION_LEVEL":0, "MSK_IPAR_MIO_FEASPUMP_LEVEL":1, "MSK_IPAR_MIO_VB_DETECTION_LEVEL":1}, verbose=verbose)
  else:
    value = prob.solve(solver=cp.GUROBI, verbose=verbose)
  if value == float('inf'):
    rospy.loginfo("  inverse shortest path MILP failed")
    return []

  # TODO: can solve every 90s, warm starting from previous solution, until optimality or time budget
  #       this way can return multiple solutions of better and better cost

  # new graph
  newGraph = graph.copy()
  changed = 0
  for j in range(len(edges)):
    edge = edges[j]
    i = edge2varnodeindex[edge[0], edge[1]]
    area = -1
    for k in range(len(allowedAreaTypes)):
      if l_.value[len(allowedAreaTypes) * i + k] == 1:
        area = allowedAreaTypes[k]
        break
    if area != graph[edge[0]][edge[1]]["area"]:
      changed += 1
    newGraph[edge[0]][edge[1]]["area"] = area
    newGraph[edge[0]][edge[1]]["cost"] = areaCosts[area]
    newGraph[edge[0]][edge[1]]["weight"] = areaCosts[area] * dist(graph.nodes[edge[0]]["point"], graph.nodes[edge[1]]["point"])
    if not newGraph.nodes[edge[0]]["portal"]:
      newGraph.nodes[edge[0]]["area"] = area
      newGraph.nodes[edge[0]]["cost"] = areaCosts[area]
    if not newGraph.nodes[edge[1]]["portal"]:
      newGraph.nodes[edge[1]]["area"] = area
      newGraph.nodes[edge[1]]["cost"] = areaCosts[area]

  # sanity check
  new_path = nx.shortest_path(newGraph, source=desiredPath[0], target=desiredPath[-1], weight="weight")
  if getCost(graph, new_path) != getCost(graph, desiredPath):
    rospy.logwarn("  new shortest path is not the desired one")
  elif verbose:
    rospy.loginfo("  inverse shortest path: success")

  # changed entries
  if verbose:
    rospy.loginfo("  changed labels: %d" % changed)

  #pdb.set_trace()
  return l_.value, newGraph


def optInvLP(graph, desiredPath):
  # Roland 2014 "Inverse multi-objective combinatorial optimization", eq. 5.10

  # variables:
  #   x_j:      indicator variable, whether edge j is part of the shortest path
  #   A_ij:     node-arc incidence matrix (rows are nodes, columns are edges) = 1 if j leaves i, -1 if j enters i, 0 otherwise
  #   b_i:      difference between entering and leaving edges = 1 if i start, -1 if i target, 0 otherwise
  #   pi_i:     dual variable
  #   lambda_j: dual variable

  # problems:
  #   SP:  min  c.x,
  #        s.t. Ax=b, x>=0
  #   ISP: min  |d-c|,
  #        s.t. sum_i a_ij * pi_i = d_j,              for all j in desired path
  #             sum_i a_ij * pi_i + lambda_j = d_j,   for all j not in desired path
  #             lambda >= 0,                          for all j not in desired path.

  verbose = False

  # auxiliary variables
  edge2index = {}
  edges = []
  weights = []
  for (i,j) in graph.edges:
    edge2index[i,j] = len(edges)
    edges.append([i,j])
    weights.append(graph[i][j]["weight"])
    edge2index[j,i] = len(edges)
    edges.append([j,i])
    weights.append(graph[j][i]["weight"])
  node2index = {}
  nodes = []
  for n in graph.nodes:
    node2index[n] = len(nodes)
    nodes.append(n)
    if n == desiredPath[0]:
      s = node2index[n]
    if n == desiredPath[-1]:
      t = node2index[n]
  weights = np.array(weights)

  # Ax = b
  A = np.zeros([len(nodes), len(edges)])
  b = np.zeros(len(nodes))
  for i in range(len(nodes)):
    for nei in graph.adj[nodes[i]]:
      j = edge2index[nodes[i], nei]
      A[i,j] = 1
      j = edge2index[nei, nodes[i]]
      A[i,j] =-1
    if i == s:
      b[i] = 1
    if i == t:
      b[i] =-1

  # optimal x
  path = nx.shortest_path(graph, source=desiredPath[0], target=desiredPath[-1], weight="weight")
  xstar = np.zeros(len(edges))
  for p in range(len(path)-1):
    j = edge2index[path[p], path[p+1]]
    xstar[j] = 1

  # desired x
  xzero = np.zeros(len(edges))
  for p in range(len(desiredPath)-1):
    j = edge2index[desiredPath[p], desiredPath[p+1]]
    xzero[j] = 1

  # inverse optimization problem
  d_ = cp.Variable(len(edges))
  pi_ = cp.Variable(len(nodes))
  lambda_ = cp.Variable(len(edges))
  cost = cp.norm1(d_ - weights)
  constraints = []
  for j in range(len(edges)):
    if xzero[j] == 1:
      constraints.append( cp.sum(cp.multiply(A[:,j], pi_)) == d_[j] )
    else:
      constraints.append( cp.sum(cp.multiply(A[:,j], pi_)) + lambda_[j] == d_[j] )
  for j in range(len(edges)):
    if xzero[j] == 0:
      constraints.append( lambda_[j] >= 0 )
  # undirected graph constraint (costs equal on both ways of each edge)
  for edge in graph.edges:
    ij = edge2index[edge[0],edge[1]]
    ji = edge2index[edge[1],edge[0]]
    constraints.append(d_[ij] == d_[ji])
  # positive edge costs (necessary for dijkstra)
  constraints.append(d_ >= 0)

  # solve with cvxpy
  prob = cp.Problem(cp.Minimize(cost), constraints)
  value = prob.solve()
  if value == float('inf'):
    rospy.loginfo("  shortest path LP failed")
    return []

  # adjusted weights
  new_weights = d_.value

  # adjusted costs
  new_costs = np.array([0.0]*len(weights))
  for j in range(len(edges)):
    edge = edges[j]
    new_costs[j] = new_weights[j] / dist(graph.nodes[edge[0]]["point"], graph.nodes[edge[1]]["point"])

  # new graph
  newGraph = graph.copy()
  for j in range(len(edges)):
    edge = edges[j]
    newGraph[edge[0]][edge[1]]["cost"] = new_costs[j]
    newGraph[edge[0]][edge[1]]["weight"] = new_weights[j]
    if not newGraph.nodes[edge[0]]["portal"]:
      newGraph.nodes[edge[0]]["cost"] = new_costs[j]
    if not newGraph.nodes[edge[1]]["portal"]:
      newGraph.nodes[edge[1]]["cost"] = new_costs[j]

  # sanity check
  new_path = nx.shortest_path(newGraph, source=desiredPath[0], target=desiredPath[-1], weight="weight")
  if getCost(graph, new_path) != getCost(graph, desiredPath):
    rospy.logwarn("  new shortest path is not the desired one")
  elif verbose:
    rospy.loginfo("  inverse shortest path: success")

  # changed entries
  if verbose:
    chent = np.sum(np.abs(new_weights - weights) > 1e-5)
    rospy.loginfo("  changed entries: %d (%d edges)" % (chent, chent/2))

  return new_weights, newGraph


def optInvLPzhang(graph, desiredPath):
  # Zhang 1996 "Calculating some inverse linear programming problems", eqs. 2.1, 2.8
  # Zhang 1996 "Calculating some inverse linear programming problems", eqs. 3.1, 3.5
  # http://jsaezgallego.com/tutorial/2017/07/16/Inverse-opitmization.html

  # TODO: need to add undirected graph constraint (costs equal on both ways of each edge)
  # TODO: check correct

  edge2index = {}
  edges = []
  weights = []
  for (i,j) in graph.edges:
    edge2index[i,j] = len(edges)
    edges.append([i,j])
    weights.append(graph[i][j]["weight"])
    edge2index[j,i] = len(edges)
    edges.append([j,i])
    weights.append(graph[j][i]["weight"])
  node2index = {}
  nodes = []
  for n in graph.nodes:
    node2index[n] = len(nodes)
    nodes.append(graph.nodes[n])

  pi = cp.Variable(len(graph.nodes))
  th = cp.Variable(len(edges))
  al = cp.Variable(len(edges))

  # J+ (in desired) and J- (not in desired)
  Jplus = np.array([0]*len(edges))
  for p in range(len(desiredPath)-1):
    Jplus[edge2index[desiredPath[p],desiredPath[p+1]]] = 1
  Jminus = 1 - Jplus

  # sum_(ij not in desired) th_ij + sum_(ij in desired) al_ij
  cost = cp.sum(cp.multiply(th, Jminus)) + cp.sum(cp.multiply(al, Jplus))

  # th_ij, al_ij >= 0
  constraints = [th >= 0, al >= 0]

  # -pi_i + pi_j - th_ij <= c_ij  for i,j not in desired
  # -pi_i + pi_j + al_ij >= c_ij  for i,j in desired
  for ij in range(len(edges)):
    i = node2index[edges[ij][0]]
    j = node2index[edges[ij][1]]
    if Jminus[ij] == 1:
      constraints.append( -pi[i] + pi[j] - th[ij] <= weights[ij] )
    elif Jplus[ij] == 1:
      constraints.append( -pi[i] + pi[j] + al[ij] >= weights[ij] )

  # solve with cvxpy
  prob = cp.Problem(cp.Minimize(cost), constraints)
  value = prob.solve()
  if value == float('inf'):
    rospy.loginfo("... inverse shortest path LP failed")
    return []

  # adjusted weights
  new_weights = np.array([0.0]*len(weights))
  for ij in range(len(edges)):
    if Jminus[ij] == 1:
      new_weights[ij] = weights[ij] + th.value[ij]
    elif Jplus[ij] == 1:
      new_weights[ij] = weights[ij] - al.value[ij]

  # adjusted costs
  new_costs = np.array([0.0]*len(weights))
  for ij in range(len(edges)):
    edge = edges[ij]
    new_costs[ij] = new_weights[ij] / dist(graph.nodes[edge[0]]["point"], graph.nodes[edge[1]]["point"])

  # new graph
  newGraph = graph.copy()
  for ij in range(len(edges)):
    edge = edges[ij]
    newGraph[edge[0]][edge[1]]["cost"] = new_costs[ij]
    newGraph[edge[0]][edge[1]]["weight"] = new_weights[ij]
    if not newGraph.nodes[edge[0]]["portal"]:
      newGraph.nodes[edge[0]]["cost"] = new_costs[ij]
    if not newGraph.nodes[edge[1]]["portal"]:
      newGraph.nodes[edge[1]]["cost"] = new_costs[ij]
  return new_weights, newGraph


def optAreaCosts(graph, areaCosts, desiredPath, badPaths):
  # variable:     ac = areaCosts                                                  # vector of all area-label costs (float)
  # cost:         (ac-areaCosts)^2
  # constraints:  sum_(j in B) dist_j * ac_j - sum_(j in S_i) dist_j * ac_j <= 0  # for all bad paths S_i
  #               ac_i >= 0                                                       # for all i

  # path cost constraints
  G = []
  h = []
  for path in badPaths:
    line = [0.0]*len(areaCosts)
    # sum_(j in B) dist_j * ac_j
    for i in range(len(desiredPath)-1):
      d = dist(graph.nodes[desiredPath[i]]["point"], graph.nodes[desiredPath[i+1]]["point"])
      if graph.nodes[desiredPath[i]]["portal"] == False:
        line[ graph.nodes[desiredPath[i]]["area"] ] += d
      elif graph.nodes[desiredPath[i+1]]["portal"] == False:
        line[ graph.nodes[desiredPath[i+1]]["area"] ] += d
    # - sum_(j in S) dist_j * ac_j
    for i in range(len(path)-1):
      d = dist(graph.nodes[path[i]]["point"], graph.nodes[path[i+1]]["point"])
      if graph.nodes[path[i]]["portal"] == False:
        line[ graph.nodes[path[i]]["area"] ] -= d
      elif graph.nodes[path[i+1]]["portal"] == False:
        line[ graph.nodes[path[i+1]]["area"] ] -= d
    G.append(line)
    h.append(0)
  G = np.array(G)
  h = np.array(h)

  # solve with cvxpy (hard constraints version ... has problems when it is impossible that the desired path is shortest)
  #x = cp.Variable(len(areaCosts))
  #cost = cp.sum_squares(x - np.array(areaCosts))
  #prob = cp.Problem(cp.Minimize(cost), [G @ x <= h, x >= 1.0])
  #value = prob.solve()

  # solve with cvxpy (soft constraints version)
  x = cp.Variable(len(areaCosts))
  cost = cp.norm2(x - np.array(areaCosts)) + 1 * cp.maximum(cp.max(G @ x - h), 0)
  prob = cp.Problem(cp.Minimize(cost), [x[0] == 0, x[1:] >= 1])
  value = prob.solve() # depending on problem and penalty weight, might have to use solver=cp.MOSEK

  # get result
  newAreaCosts = x.value
  if value == float('inf'):
    rospy.loginfo("... optimization failed")
    return x.value, graph.copy()

  # new graph
  newGraph = graph.copy()
  for node in newGraph:
    if newGraph.nodes[node]["area"] != -1:
      newGraph.nodes[node]["cost"] = newAreaCosts[ newGraph.nodes[node]["area"] ]
  for edge in list(newGraph.edges):
    if newGraph.nodes[edge[0]]["area"] != -1:
      area = newGraph.nodes[edge[0]]["area"]
    else:
      area = newGraph.nodes[edge[1]]["area"]
    newGraph[edge[0]][edge[1]]["area"] = area
    newGraph[edge[0]][edge[1]]["cost"] = newAreaCosts[area]
    newGraph[edge[0]][edge[1]]["weight"] = newAreaCosts[area] * dist(newGraph.nodes[edge[0]]["point"], newGraph.nodes[edge[1]]["point"])
  return x.value, newGraph


def optPolyCosts(graph, desiredPath, badPaths):
  # variable:     nc = nodeCosts                                                  # vector of all node costs (float)
  # cost:         (nc-nodeCosts)^2
  # constraints:  sum_(j in B) dist_j * nc_j - sum_(j in S_i) dist_j * nc_j <= 0  # for all paths S_i
  #               nc_i >= 0                                                       # for all i

  # define variable
  nodeCosts = np.array([0.0]*len(graph.nodes))
  for node in graph.nodes:
    nodeCosts[ graph.nodes[node]["id"] ] = graph.nodes[node]["cost"]

  # path cost constraints
  G = []
  h = []
  for path in badPaths:
    line = [0.0]*len(nodeCosts)
    # sum_(j in B) dist_j * nc_j
    for i in range(len(desiredPath)-1):
      d = dist(graph.nodes[desiredPath[i]]["point"], graph.nodes[desiredPath[i+1]]["point"])
      if graph.nodes[desiredPath[i]]["portal"] == False:
        line[ graph.nodes[desiredPath[i]]["id"] ] += d
      elif graph.nodes[desiredPath[i+1]]["portal"] == False:
        line[ graph.nodes[desiredPath[i+1]]["id"] ] += d
    # - sum_(j in S) dist_j * nc_j
    for i in range(len(path)-1):
      d = dist(graph.nodes[path[i]]["point"], graph.nodes[path[i+1]]["point"])
      if graph.nodes[path[i]]["portal"] == False:
        line[ graph.nodes[path[i]]["id"] ] -= d
      elif graph.nodes[path[i+1]]["portal"] == False:
        line[ graph.nodes[path[i+1]]["id"] ] -= d
    G.append(line)
    h.append(0)
  G = np.array(G)
  h = np.array(h)

  # get lower bound on edge costs
  mincost = float("inf")
  for edge in list(graph.edges):
    if graph[edge[0]][edge[1]]["weight"] < mincost:
      mincost = graph[edge[0]][edge[1]]["weight"]

  # solve with cvxpy (hard constraints version ... has problems when it is impossible that the desired path is shortest)
  #x = cp.Variable(len(graph.nodes))
  #cost = cp.sum_squares(x - np.array(nodeCosts))
  #prob = cp.Problem(cp.Minimize(cost), [G @ x <= h, x >= mincost])
  #prob.solve()

  # solve with cvxpy (soft constraints version)
  x = cp.Variable(len(graph.nodes))
  cost = cp.norm1(x - np.array(nodeCosts)) + 1 * cp.maximum(cp.max(G @ x - h), 0)
  prob = cp.Problem(cp.Minimize(cost), [x >= mincost])
  prob.solve() # depending on problem and penalty weight, might have to use solver=cp.MOSEK

  # get result
  newPolyCosts = x.value

  # new graph
  newGraph = graph.copy()
  for node in newGraph:
    newGraph.nodes[node]["cost"] = newPolyCosts[ newGraph.nodes[node]["id"] ]
  for edge in list(newGraph.edges):
    if newGraph.nodes[edge[0]]["cost"] != -1:
      cost = newGraph.nodes[edge[0]]["cost"]
    else:
      cost = newGraph.nodes[edge[1]]["cost"]
    newGraph[edge[0]][edge[1]]["cost"] = cost
    newGraph[edge[0]][edge[1]]["weight"] = cost * dist(newGraph.nodes[edge[0]]["point"], newGraph.nodes[edge[1]]["point"])
  return x.value, newGraph


def optPolyLabelsApproxFromCosts(graph, areaCosts, allowedAreaTypes, desiredPath, badPaths):
  # get optimal node costs
  nodeCosts,tmpGraph = optPolyCosts(graph, desiredPath, badPaths)

  # infer each node's area-type from cost
  newPolyLabels = [0]*len(graph.nodes)
  newGraph = graph.copy()
  for node in graph.nodes:
    oldarea = graph.nodes[node]["area"]
    oldcost = graph.nodes[node]["cost"]
    newcost = nodeCosts[ graph.nodes[node]["id"] ]
    # if cost has changed then choose area type with closest cost
    if abs(newcost - oldcost) > 0.01:
      bestArea = -1
      bestDist = float('inf')
      for a in allowedAreaTypes:
        if a == oldarea:
          continue
        d = abs(newcost - areaCosts[a])
        if d < bestDist:
          bestDist = d
          bestArea = a
      newGraph.nodes[node]["area"] = bestArea
      newGraph.nodes[node]["cost"] = areaCosts[bestArea]
      newPolyLabels[ newGraph.nodes[node]["id"] ] = bestArea
    else:
      newPolyLabels[ newGraph.nodes[node]["id"] ] = oldarea
  for edge in list(newGraph.edges):
    if newGraph.nodes[edge[0]]["cost"] != -1:
      cost = newGraph.nodes[edge[0]]["cost"]
      area = newGraph.nodes[edge[0]]["area"]
    else:
      cost = newGraph.nodes[edge[1]]["cost"]
      area = newGraph.nodes[edge[1]]["area"]
    newGraph[edge[0]][edge[1]]["area"] = area
    newGraph[edge[0]][edge[1]]["cost"] = cost
    newGraph[edge[0]][edge[1]]["weight"] = cost * dist(newGraph.nodes[edge[0]]["point"], newGraph.nodes[edge[1]]["point"])
  return newPolyLabels, newGraph


def optPolyLabelsApproxFromCosts2(graph, areaCosts, allowedAreaTypes, desiredPath, badPaths, graphOptCosts):
  # infer each node's area-type from cost
  newPolyLabels = [0]*len(graph.nodes)
  newGraph = graph.copy()
  for node in graph.nodes:
    oldarea = graph.nodes[node]["area"]
    oldcost = graph.nodes[node]["cost"]
    newcost = graphOptCosts.nodes[node]["cost"]
    if abs(newcost - oldcost) > 0.01:
      # if cost has changed then choose area type with closest cost
      bestArea = -1
      bestDist = float('inf')
      for a in allowedAreaTypes:
        if a == oldarea:
          continue
        d = abs(newcost - areaCosts[a])
        if d < bestDist:
          bestDist = d
          bestArea = a
      newGraph.nodes[node]["area"] = bestArea
      newGraph.nodes[node]["cost"] = areaCosts[bestArea]
      newPolyLabels[ newGraph.nodes[node]["id"] ] = bestArea
    else:
      newPolyLabels[ newGraph.nodes[node]["id"] ] = oldarea
  for edge in list(newGraph.edges):
    if newGraph.nodes[edge[0]]["cost"] != -1:
      cost = newGraph.nodes[edge[0]]["cost"]
      area = newGraph.nodes[edge[0]]["area"]
    else:
      cost = newGraph.nodes[edge[1]]["cost"]
      area = newGraph.nodes[edge[1]]["area"]
    newGraph[edge[0]][edge[1]]["area"] = area
    newGraph[edge[0]][edge[1]]["cost"] = cost
    newGraph[edge[0]][edge[1]]["weight"] = cost * dist(newGraph.nodes[edge[0]]["point"], newGraph.nodes[edge[1]]["point"])
  return newPolyLabels, newGraph


def optPolyLabels(graph, areaCosts, desiredPath, badPaths):
  # variable:     l = nodeLabelsHotEnc                                      # vector of all node area-labels using one-hot encoding (bool)
  # cost:         (l-nodeLabelsHotEnc)^2
  # constraints:  edgeCost_i = dist_i * ac_0 * l_0 + dist_i * ac_1 * l_1    # for all edges i
  #               sum_(j in B) edgeCost_j - sum_(j in S_i) edgeCost_j <= 0  # for all paths S_i
  #               l_i + l_(i+1) = 1                                         # each node can have only one 1

  # define variable
  nodeLabelsHotEnc = np.array([0]*len(graph.nodes)*2)
  for node in graph.nodes:
    if graph.nodes[node]["area"] == -1:
      continue
    elif graph.nodes[node]["area"] == 1:
      nodeLabelsHotEnc[ graph.nodes[node]["id"]*2   ] = 1
    elif graph.nodes[node]["area"] == 2:
      nodeLabelsHotEnc[ graph.nodes[node]["id"]*2+1 ] = 1
    else:
      print("ERROR: unexpected value")
      return []

  # path cost constraints
  G = []
  h = []
  for path in badPaths:
    line = [0.0]*len(nodeLabelsHotEnc)
    # sum_(i in B) dist_i * ac_0 * l_0 + dist_i * ac_1 * l_1
    for i in range(len(desiredPath)-1):
      d = dist(graph.nodes[desiredPath[i]]["point"], graph.nodes[desiredPath[i+1]]["point"])
      if graph.nodes[desiredPath[i]]["portal"] == False:
        line[ graph.nodes[desiredPath[i]]["id"]*2   ] += d * areaCosts[1]
        line[ graph.nodes[desiredPath[i]]["id"]*2+1 ] += d * areaCosts[2]
      elif graph.nodes[desiredPath[i+1]]["portal"] == False:
        line[ graph.nodes[desiredPath[i+1]]["id"]*2   ] += d * areaCosts[1]
        line[ graph.nodes[desiredPath[i+1]]["id"]*2+1 ] += d * areaCosts[2]
    # - sum_(i in S) dist_i * ac_0 * l_0 + dist_i * ac_1 * l_1
    for i in range(len(path)-1):
      d = dist(graph.nodes[path[i]]["point"], graph.nodes[path[i+1]]["point"])
      if graph.nodes[path[i]]["portal"] == False:
        line[ graph.nodes[path[i]]["id"]*2   ] -= d * areaCosts[1]
        line[ graph.nodes[path[i]]["id"]*2+1 ] -= d * areaCosts[2]
      elif graph.nodes[path[i+1]]["portal"] == False:
        line[ graph.nodes[path[i+1]]["id"]*2   ] -= d * areaCosts[1]
        line[ graph.nodes[path[i+1]]["id"]*2+1 ] -= d * areaCosts[2]
    G.append(line)
    h.append(0)
  G = np.array(G)
  h = np.array(h)

  # single active label per node
  A = []
  b = []
  for i in range(len(graph.nodes)):
    line = [0]*len(nodeLabelsHotEnc)
    line[2*i  ] = 1
    line[2*i+1] = 1
    A.append(line)
    b.append(1)
  A = np.array(A)
  b = np.array(b)

  # solve with cvxpy (hard constraints version ... has problems when it is impossible that the desired path is shortest)
  #x = cp.Variable(len(nodeLabelsHotEnc), boolean=True)
  #cost = cp.sum_squares(x - np.array(nodeLabelsHotEnc))
  #prob = cp.Problem(cp.Minimize(cost), [G @ x <= h, A @ x == b])
  #prob.solve(solver=cp.MOSEK, verbose=True, mosek_params={"MSK_DPAR_MIO_MAX_TIME":300})

  # solve with cvxpy (soft constraints version)
  x = cp.Variable(len(nodeLabelsHotEnc), boolean=True)
  cost = cp.norm1(x - np.array(nodeLabelsHotEnc)) + 1 * cp.maximum(cp.max(G @ x - h), 0)
  #cost = cp.norm1(x - np.array(nodeLabelsHotEnc)) + 0.1 * cp.sum(G @ x - h)
  prob = cp.Problem(cp.Minimize(cost), [A @ x == b])
  prob.solve(solver=cp.MOSEK, mosek_params={"MSK_DPAR_MIO_MAX_TIME":90})

  # get new poly labels and graph
  newPolyLabels = [0]*len(graph.nodes)
  newGraph = graph.copy()
  for i in range(len(newGraph.nodes)):
    if x.value[i*2]:
      area = 1
    elif x.value[i*2+1]:
      area = 2
    else:
      area = -1
    newPolyLabels[i] = area
    for node in newGraph:
      if newGraph.nodes[node]["id"] == i:
        if newGraph.nodes[node]["area"] != -1:
          if area == -1:
            print("WARNING: new area -1 but original is not...")
            pdb.set_trace()
          newGraph.nodes[node]["area"] = area
          newGraph.nodes[node]["cost"] = areaCosts[area]
        break
  for edge in list(newGraph.edges):
    if newGraph.nodes[edge[0]]["cost"] != -1:
      cost = newGraph.nodes[edge[0]]["cost"]
      area = newGraph.nodes[edge[0]]["area"]
    else:
      cost = newGraph.nodes[edge[1]]["cost"]
      area = newGraph.nodes[edge[1]]["area"]
    newGraph[edge[0]][edge[1]]["area"] = area
    newGraph[edge[0]][edge[1]]["cost"] = cost
    newGraph[edge[0]][edge[1]]["weight"] = cost * dist(newGraph.nodes[edge[0]]["point"], newGraph.nodes[edge[1]]["point"])
  return newPolyLabels, newGraph


def getPolyLabelsAndGraph(x, graph, nodeList, areaCosts):
  # get new poly labels and graph
  newPolyLabels = [0]*len(graph.nodes)
  newGraph = graph.copy()
  for node in graph:
    newPolyLabels[ graph.nodes[node]["id"] ] = graph.nodes[node]["area"]
  for i in range(len(nodeList)):
    node = nodeList[i]
    if x.value[i*2]:
      area = 1
    elif x.value[i*2+1]:
      area = 2
    else:
      area = -1
    newPolyLabels[ graph.nodes[node]["id"] ] = area
    if newGraph.nodes[node]["area"] != -1:
      if area == -1:
        print("WARNING: new area -1 but original is not...")
        pdb.set_trace()
      newGraph.nodes[node]["area"] = area
      newGraph.nodes[node]["cost"] = areaCosts[area]
  for edge in list(newGraph.edges):
    if newGraph.nodes[edge[0]]["cost"] != -1:
      cost = newGraph.nodes[edge[0]]["cost"]
      area = newGraph.nodes[edge[0]]["area"]
    else:
      cost = newGraph.nodes[edge[1]]["cost"]
      area = newGraph.nodes[edge[1]]["area"]
    newGraph[edge[0]][edge[1]]["area"] = area
    newGraph[edge[0]][edge[1]]["cost"] = cost
    newGraph[edge[0]][edge[1]]["weight"] = cost * dist(newGraph.nodes[edge[0]]["point"], newGraph.nodes[edge[1]]["point"])
  return newPolyLabels, newGraph


def optPolyLabelsInPath(graph, areaCosts, desiredPath, badPaths):
  # variable:     l = nodeLabelsHotEnc                                      # vector of *in-path* node area-labels using one-hot encoding (bool)
  # cost:         (l-nodeLabelsHotEnc)^2
  # constraints:  edgeCost_i = dist_i * ac_0 * l_0 + dist_i * ac_1 * l_1    # for all edges i
  #               sum_(j in B) edgeCost_j - sum_(j in S_i) edgeCost_j <= 0  # for all paths S_i
  #               l_i + l_(i+1) = 1                                         # each node can have only one 1

  auxgraph = graph.copy()

  # define variable
  nodeLabelsHotEnc = []
  nodeList = []
  for node in graph.nodes:
    if graph.nodes[node]["area"] == -1:
      continue
    # check if this node is in any of the paths
    inpath = False
    if node in desiredPath:
      inpath = True
    if not inpath:
      for badPath in badPaths:
        if node in badPath:
          inpath = True
          break
    # add variables
    if inpath:
      if graph.nodes[node]["area"] == 1:
        auxgraph.nodes[node]["varidx"] = len(nodeLabelsHotEnc)
        nodeLabelsHotEnc.append(1)
        nodeLabelsHotEnc.append(0)
        nodeList.append(node)
      elif graph.nodes[node]["area"] == 2:
        auxgraph.nodes[node]["varidx"] = len(nodeLabelsHotEnc)
        nodeLabelsHotEnc.append(0)
        nodeLabelsHotEnc.append(1)
        nodeList.append(node)
  nodeLabelsHotEnc = np.array(nodeLabelsHotEnc)

  # path cost constraints
  G = []
  h = []
  for path in badPaths:
    line = [0.0]*len(nodeLabelsHotEnc)
    # sum_(i in B) dist_i * ac_0 * l_0 + dist_i * ac_1 * l_1
    for i in range(len(desiredPath)-1):
      d = dist(graph.nodes[desiredPath[i]]["point"], graph.nodes[desiredPath[i+1]]["point"])
      if graph.nodes[desiredPath[i]]["portal"] == False:
        line[ auxgraph.nodes[desiredPath[i]]["varidx"]   ] += d * areaCosts[1]
        line[ auxgraph.nodes[desiredPath[i]]["varidx"]+1 ] += d * areaCosts[2]
      elif graph.nodes[desiredPath[i+1]]["portal"] == False:
        line[ auxgraph.nodes[desiredPath[i+1]]["varidx"]   ] += d * areaCosts[1]
        line[ auxgraph.nodes[desiredPath[i+1]]["varidx"]+1 ] += d * areaCosts[2]
    # - sum_(i in S) dist_i * ac_0 * l_0 + dist_i * ac_1 * l_1
    for i in range(len(path)-1):
      d = dist(graph.nodes[path[i]]["point"], graph.nodes[path[i+1]]["point"])
      if graph.nodes[path[i]]["portal"] == False:
        line[ auxgraph.nodes[path[i]]["varidx"]   ] -= d * areaCosts[1]
        line[ auxgraph.nodes[path[i]]["varidx"]+1 ] -= d * areaCosts[2]
      elif graph.nodes[path[i+1]]["portal"] == False:
        line[ auxgraph.nodes[path[i+1]]["varidx"]   ] -= d * areaCosts[1]
        line[ auxgraph.nodes[path[i+1]]["varidx"]+1 ] -= d * areaCosts[2]
    G.append(line)
    h.append(0)
  G = np.array(G)
  h = np.array(h)

  # single active label per node
  A = []
  b = []
  for i in range(len(nodeList)):
    line = [0]*len(nodeLabelsHotEnc)
    line[2*i  ] = 1
    line[2*i+1] = 1
    A.append(line)
    b.append(1)
  A = np.array(A)
  b = np.array(b)

  # weight (area, not-on-path-penalty)
  weights = [1]*len(nodeLabelsHotEnc)
  if False:
    for i in range(len(desiredPath)):
      if graph.nodes[desiredPath[i]]["portal"] == False:
        if i == 0:
          d = dist(graph.nodes[desiredPath[i]]["point"], graph.nodes[desiredPath[i+1]]["point"])
          weights[ auxgraph.nodes[desiredPath[i]]["varidx"]   ] = d
          weights[ auxgraph.nodes[desiredPath[i]]["varidx"]+1 ] = d
        elif i == len(desiredPath)-1:
          d = dist(graph.nodes[desiredPath[i-1]]["point"], graph.nodes[desiredPath[i]]["point"])
          weights[ auxgraph.nodes[desiredPath[i]]["varidx"]   ] = d
          weights[ auxgraph.nodes[desiredPath[i]]["varidx"]+1 ] = d
        else:
          d1 = dist(graph.nodes[desiredPath[i-1]]["point"], graph.nodes[desiredPath[i]]["point"])
          d2 = dist(graph.nodes[desiredPath[i]]["point"], graph.nodes[desiredPath[i+1]]["point"])
          weights[ auxgraph.nodes[desiredPath[i]]["varidx"]   ] = d1 + d2
          weights[ auxgraph.nodes[desiredPath[i]]["varidx"]+1 ] = d1 + d2
    for path in badPaths:
      for i in range(len(path)):
        if graph.nodes[path[i]]["portal"] == False:
          if i == 0:
            d = dist(graph.nodes[path[i]]["point"], graph.nodes[path[i+1]]["point"])
            weights[ auxgraph.nodes[path[i]]["varidx"]   ] = d
            weights[ auxgraph.nodes[path[i]]["varidx"]+1 ] = d
          elif i == len(path)-1:
            d = dist(graph.nodes[path[i-1]]["point"], graph.nodes[path[i]]["point"])
            weights[ auxgraph.nodes[path[i]]["varidx"]   ] = d
            weights[ auxgraph.nodes[path[i]]["varidx"]+1 ] = d
          else:
            d1 = dist(graph.nodes[path[i-1]]["point"], graph.nodes[path[i]]["point"])
            d2 = dist(graph.nodes[path[i]]["point"], graph.nodes[path[i+1]]["point"])
            weights[ auxgraph.nodes[path[i]]["varidx"]   ] = d1 + d2
            weights[ auxgraph.nodes[path[i]]["varidx"]+1 ] = d1 + d2
  #weights = [10]*len(nodeLabelsHotEnc)
  for i in range(len(nodeList)):
    if nodeList[i] not in desiredPath[1:-1]:
      weights[2*i  ] *= 10
      weights[2*i+1] *= 10

  # solve with cvxpy (hard constraints version ... has problems when it is impossible that the desired path is shortest)
  #x = cp.Variable(len(nodeLabelsHotEnc), boolean=True)
  #cost = cp.sum_squares(x - np.array(nodeLabelsHotEnc))
  #prob = cp.Problem(cp.Minimize(cost), [G @ x <= h, A @ x == b])
  #prob.solve(solver=cp.MOSEK, verbose=True, mosek_params={"MSK_DPAR_MIO_MAX_TIME":300})

  # solve with cvxpy (soft constraints version)
  x = cp.Variable(len(nodeLabelsHotEnc), boolean=True)
  cost = cp.norm1(cp.multiply(x - nodeLabelsHotEnc, weights)) + 1 * cp.maximum(cp.max(G @ x - h), 0)  # requires many iterations or many badPaths, always has few changes and low (but non-zero) error
  #cost = cp.norm1(x - nodeLabelsHotEnc) + 1 * cp.maximum(cp.max(G @ x - h), 0)  # requires many iterations or many badPaths, always has few changes and low (but non-zero) error
  #cost = cp.norm1(x - nodeLabelsHotEnc) + 0.1 * cp.sum(G @ x - h)              # too many changed variables (so that cost of desired is as low as possible) but zero error
  prob = cp.Problem(cp.Minimize(cost), [A @ x == b])
  prob.solve(solver=cp.MOSEK, mosek_params={"MSK_DPAR_MIO_MAX_TIME":90})

  return getPolyLabelsAndGraph(x, graph, nodeList, areaCosts)


#def optPolyLabelsInPathItLog(graph, areaCosts, desiredPath, badPaths):
#  https://www.cvxpy.org/examples/applications/sparse_solution.html


def optPolyLabelsInPathTradeoff(graph, areaCosts, desiredPath, badPaths):
  # variable:     l = nodeLabelsHotEnc                                      # vector of *in-path* node area-labels using one-hot encoding (bool)
  # cost:         (l-nodeLabelsHotEnc)^2
  # constraints:  edgeCost_i = dist_i * ac_0 * l_0 + dist_i * ac_1 * l_1    # for all edges i
  #               sum_(j in B) edgeCost_j - sum_(j in S_i) edgeCost_j <= 0  # for all paths S_i
  #               l_i + l_(i+1) = 1                                         # each node can have only one 1

  auxgraph = graph.copy()

  # define variable
  nodeLabelsHotEnc = []
  nodeList = []
  for node in graph.nodes:
    if graph.nodes[node]["area"] == -1:
      continue
    # check if this node is in any of the paths
    inpath = False
    if node in desiredPath:
      inpath = True
    if not inpath:
      for badPath in badPaths:
        if node in badPath:
          inpath = True
          break
    # add variables
    if inpath:
      if graph.nodes[node]["area"] == 1:
        auxgraph.nodes[node]["varidx"] = len(nodeLabelsHotEnc)
        nodeLabelsHotEnc.append(1)
        nodeLabelsHotEnc.append(0)
        nodeList.append(node)
      elif graph.nodes[node]["area"] == 2:
        auxgraph.nodes[node]["varidx"] = len(nodeLabelsHotEnc)
        nodeLabelsHotEnc.append(0)
        nodeLabelsHotEnc.append(1)
        nodeList.append(node)
  nodeLabelsHotEnc = np.array(nodeLabelsHotEnc)

  # path cost constraints
  G = []
  h = []
  for path in badPaths:
    line = [0.0]*len(nodeLabelsHotEnc)
    # sum_(i in B) dist_i * ac_0 * l_0 + dist_i * ac_1 * l_1
    for i in range(len(desiredPath)-1):
      d = dist(graph.nodes[desiredPath[i]]["point"], graph.nodes[desiredPath[i+1]]["point"])
      if graph.nodes[desiredPath[i]]["portal"] == False:
        line[ auxgraph.nodes[desiredPath[i]]["varidx"]   ] += d * areaCosts[1]
        line[ auxgraph.nodes[desiredPath[i]]["varidx"]+1 ] += d * areaCosts[2]
      elif graph.nodes[desiredPath[i+1]]["portal"] == False:
        line[ auxgraph.nodes[desiredPath[i+1]]["varidx"]   ] += d * areaCosts[1]
        line[ auxgraph.nodes[desiredPath[i+1]]["varidx"]+1 ] += d * areaCosts[2]
    # - sum_(i in S) dist_i * ac_0 * l_0 + dist_i * ac_1 * l_1
    for i in range(len(path)-1):
      d = dist(graph.nodes[path[i]]["point"], graph.nodes[path[i+1]]["point"])
      if graph.nodes[path[i]]["portal"] == False:
        line[ auxgraph.nodes[path[i]]["varidx"]   ] -= d * areaCosts[1]
        line[ auxgraph.nodes[path[i]]["varidx"]+1 ] -= d * areaCosts[2]
      elif graph.nodes[path[i+1]]["portal"] == False:
        line[ auxgraph.nodes[path[i+1]]["varidx"]   ] -= d * areaCosts[1]
        line[ auxgraph.nodes[path[i+1]]["varidx"]+1 ] -= d * areaCosts[2]
    G.append(line)
    h.append(0)
  G = np.array(G)
  h = np.array(h)

  # single active label per node
  A = []
  b = []
  for i in range(len(nodeList)):
    line = [0]*len(nodeLabelsHotEnc)
    line[2*i  ] = 1
    line[2*i+1] = 1
    A.append(line)
    b.append(1)
  A = np.array(A)
  b = np.array(b)

  # weights (by area, and penalize not-on-path)
  weights = [1]*len(nodeLabelsHotEnc)
  if False:
    for i in range(len(desiredPath)):
      if graph.nodes[desiredPath[i]]["portal"] == False:
        if i == 0:
          d = dist(graph.nodes[desiredPath[i]]["point"], graph.nodes[desiredPath[i+1]]["point"])
          weights[ auxgraph.nodes[desiredPath[i]]["varidx"]   ] = d
          weights[ auxgraph.nodes[desiredPath[i]]["varidx"]+1 ] = d
        elif i == len(desiredPath)-1:
          d = dist(graph.nodes[desiredPath[i-1]]["point"], graph.nodes[desiredPath[i]]["point"])
          weights[ auxgraph.nodes[desiredPath[i]]["varidx"]   ] = d
          weights[ auxgraph.nodes[desiredPath[i]]["varidx"]+1 ] = d
        else:
          d1 = dist(graph.nodes[desiredPath[i-1]]["point"], graph.nodes[desiredPath[i]]["point"])
          d2 = dist(graph.nodes[desiredPath[i]]["point"], graph.nodes[desiredPath[i+1]]["point"])
          weights[ auxgraph.nodes[desiredPath[i]]["varidx"]   ] = d1 + d2
          weights[ auxgraph.nodes[desiredPath[i]]["varidx"]+1 ] = d1 + d2
    for path in badPaths:
      for i in range(len(path)):
        if graph.nodes[path[i]]["portal"] == False:
          if i == 0:
            d = dist(graph.nodes[path[i]]["point"], graph.nodes[path[i+1]]["point"])
            weights[ auxgraph.nodes[path[i]]["varidx"]   ] = d
            weights[ auxgraph.nodes[path[i]]["varidx"]+1 ] = d
          elif i == len(path)-1:
            d = dist(graph.nodes[path[i-1]]["point"], graph.nodes[path[i]]["point"])
            weights[ auxgraph.nodes[path[i]]["varidx"]   ] = d
            weights[ auxgraph.nodes[path[i]]["varidx"]+1 ] = d
          else:
            d1 = dist(graph.nodes[path[i-1]]["point"], graph.nodes[path[i]]["point"])
            d2 = dist(graph.nodes[path[i]]["point"], graph.nodes[path[i+1]]["point"])
            weights[ auxgraph.nodes[path[i]]["varidx"]   ] = d1 + d2
            weights[ auxgraph.nodes[path[i]]["varidx"]+1 ] = d1 + d2
  #weights = [10]*len(nodeLabelsHotEnc)
  for i in range(len(nodeList)):
    if nodeList[i] not in desiredPath[1:-1]:
      weights[2*i  ] *= 10
      weights[2*i+1] *= 10

  # solve with cvxpy (hard constraints version ... has problems when it is impossible that the desired path is shortest)
  #x = cp.Variable(len(nodeLabelsHotEnc), boolean=True)
  #cost = cp.sum_squares(x - np.array(nodeLabelsHotEnc))
  #prob = cp.Problem(cp.Minimize(cost), [G @ x <= h, A @ x == b])
  #prob.solve(solver=cp.MOSEK, verbose=True, mosek_params={"MSK_DPAR_MIO_MAX_TIME":300})

  # solve with cvxpy (soft constraints version)
  x = cp.Variable(len(nodeLabelsHotEnc), boolean=True)
  #cost = cp.norm1(x - nodeLabelsHotEnc) + 1 * cp.maximum(cp.max(G @ x - h), 0)  # requires many iterations or many badPaths, always has few changes and low (but non-zero) error
  #cost = cp.norm1(x - nodeLabelsHotEnc) + 0.1 * cp.sum(G @ x - h)              # too many changed variables (so that cost of desired is as low as possible) but zero error
  #prob = cp.Problem(cp.Minimize(cost), [A @ x == b])
  #prob.solve(solver=cp.MOSEK, mosek_params={"MSK_DPAR_MIO_MAX_TIME":90})

  # costG ub
  cost = cp.norm1(cp.multiply(x - nodeLabelsHotEnc, weights))
  prob = cp.Problem(cp.Minimize(cost), [A @ x == b])
  prob.solve(solver=cp.MOSEK, mosek_params={"MSK_DPAR_MIO_MAX_TIME":90})
  costG_ub = np.sum(G @ x.value - h)

  # costG lb
  cost = cp.sum(G @ x - h)
  prob = cp.Problem(cp.Minimize(cost), [A @ x == b])
  prob.solve(solver=cp.MOSEK, mosek_params={"MSK_DPAR_MIO_MAX_TIME":90})
  costG_lb = np.sum(G @ x.value - h)

  # trade-off curve
  tradeoffs = []
  curve_dist = []
  curve_l1 = []
  curve_newgraph = []
  curve_newpolylabels = []
  for tradeoff in np.linspace(0,1,11):
    # solve with desired trade-off
    cost = cp.norm1(cp.multiply(x - nodeLabelsHotEnc, weights))
    prob = cp.Problem(cp.Minimize(cost), [A @ x == b, cp.sum(G @ x - h) <= costG_lb + (costG_ub - costG_lb) * tradeoff])
    result = prob.solve(solver=cp.MOSEK, mosek_params={"MSK_DPAR_MIO_MAX_TIME":90})
    if result == float('inf'):
      rospy.loginfo("failed")
      continue
    # simplify...
    if True:
      h2 = G @ x.value
      h2[G @ x.value <= 0] = 0
      cost = cp.norm1(cp.multiply(x - nodeLabelsHotEnc, weights))
      prob = cp.Problem(cp.Minimize(cost), [A @ x == b, G @ x <= h2])
      result = prob.solve(solver=cp.MOSEK, mosek_params={"MSK_DPAR_MIO_MAX_TIME":90})
    # get new graph/labels
    newPolyLabels, newGraph = getPolyLabelsAndGraph(x, graph, nodeList, areaCosts)
    # save
    curve_newgraph.append(newGraph)
    curve_newpolylabels.append(newPolyLabels)
    # save tradeoff
    tradeoffs.append(tradeoff)
    # get distance
    path = nx.shortest_path(newGraph, source=desiredPath[0], target=desiredPath[-1], weight="weight")
    curve_dist.append(pathDistance(desiredPath, path))
    # get L1norm
    curve_l1.append(np.linalg.norm(x.value - nodeLabelsHotEnc, 1))

  # plot curve
  plt.figure(1)
  plt.clf()

  plt.subplot(121)
  plt.plot(tradeoffs, curve_dist, 'r', label="distance")
  plt.plot(tradeoffs, curve_l1, 'b', label="L1")
  plt.legend()
  plt.xlabel("alpha")
  plt.ylabel("distance")

  plt.subplot(122)
  plt.plot(curve_l1, curve_dist)
  plt.xlabel("L1")
  plt.ylabel("distance")

  #plt.draw()
  #plt.pause(0.001)
  plt.show()

  # best solution
  mindist = min(curve_dist)
  idx = np.where(curve_dist <= mindist)[0][-1]
  return curve_newpolylabels[idx], curve_newgraph[idx]


def optPolyLabelsInPathTradeoff2(graph, areaCosts, desiredPath, badPaths):
  # variable:     l = nodeLabelsHotEnc                                      # vector of *in-path* node area-labels using one-hot encoding (bool)
  # cost:         (l-nodeLabelsHotEnc)^2
  # constraints:  edgeCost_i = dist_i * ac_0 * l_0 + dist_i * ac_1 * l_1    # for all edges i
  #               sum_(j in B) edgeCost_j - sum_(j in S_i) edgeCost_j <= 0  # for all paths S_i
  #               l_i + l_(i+1) = 1                                         # each node can have only one 1

  auxgraph = graph.copy()

  # define variable
  nodeLabelsHotEnc = []
  nodeList = []
  for node in graph.nodes:
    if graph.nodes[node]["area"] == -1:
      continue
    # check if this node is in any of the paths
    inpath = False
    if node in desiredPath:
      inpath = True
    if not inpath:
      for badPath in badPaths:
        if node in badPath:
          inpath = True
          break
    # add variables
    if inpath:
      if graph.nodes[node]["area"] == 1:
        auxgraph.nodes[node]["varidx"] = len(nodeLabelsHotEnc)
        nodeLabelsHotEnc.append(1)
        nodeLabelsHotEnc.append(0)
        nodeList.append(node)
      elif graph.nodes[node]["area"] == 2:
        auxgraph.nodes[node]["varidx"] = len(nodeLabelsHotEnc)
        nodeLabelsHotEnc.append(0)
        nodeLabelsHotEnc.append(1)
        nodeList.append(node)
  nodeLabelsHotEnc = np.array(nodeLabelsHotEnc)

  # path cost constraints
  G = []
  h = []
  for path in badPaths:
    line = [0.0]*len(nodeLabelsHotEnc)
    # sum_(i in B) dist_i * ac_0 * l_0 + dist_i * ac_1 * l_1
    for i in range(len(desiredPath)-1):
      d = dist(graph.nodes[desiredPath[i]]["point"], graph.nodes[desiredPath[i+1]]["point"])
      if graph.nodes[desiredPath[i]]["portal"] == False:
        line[ auxgraph.nodes[desiredPath[i]]["varidx"]   ] += d * areaCosts[1]
        line[ auxgraph.nodes[desiredPath[i]]["varidx"]+1 ] += d * areaCosts[2]
      elif graph.nodes[desiredPath[i+1]]["portal"] == False:
        line[ auxgraph.nodes[desiredPath[i+1]]["varidx"]   ] += d * areaCosts[1]
        line[ auxgraph.nodes[desiredPath[i+1]]["varidx"]+1 ] += d * areaCosts[2]
    # - sum_(i in S) dist_i * ac_0 * l_0 + dist_i * ac_1 * l_1
    for i in range(len(path)-1):
      d = dist(graph.nodes[path[i]]["point"], graph.nodes[path[i+1]]["point"])
      if graph.nodes[path[i]]["portal"] == False:
        line[ auxgraph.nodes[path[i]]["varidx"]   ] -= d * areaCosts[1]
        line[ auxgraph.nodes[path[i]]["varidx"]+1 ] -= d * areaCosts[2]
      elif graph.nodes[path[i+1]]["portal"] == False:
        line[ auxgraph.nodes[path[i+1]]["varidx"]   ] -= d * areaCosts[1]
        line[ auxgraph.nodes[path[i+1]]["varidx"]+1 ] -= d * areaCosts[2]
    G.append(line)
    h.append(0)
  G = np.array(G)
  h = np.array(h)

  # single active label per node
  A = []
  b = []
  for i in range(len(nodeList)):
    line = [0]*len(nodeLabelsHotEnc)
    line[2*i  ] = 1
    line[2*i+1] = 1
    A.append(line)
    b.append(1)
  A = np.array(A)
  b = np.array(b)

  # solve with cvxpy (hard constraints version ... has problems when it is impossible that the desired path is shortest)
  #x = cp.Variable(len(nodeLabelsHotEnc), boolean=True)
  #cost = cp.sum_squares(x - np.array(nodeLabelsHotEnc))
  #prob = cp.Problem(cp.Minimize(cost), [G @ x <= h, A @ x == b])
  #prob.solve(solver=cp.MOSEK, verbose=True, mosek_params={"MSK_DPAR_MIO_MAX_TIME":300})

  # solve with cvxpy (soft constraints version)
  x = cp.Variable(len(nodeLabelsHotEnc), boolean=True)
  #cost = cp.norm1(x - nodeLabelsHotEnc) + 1 * cp.maximum(cp.max(G @ x - h), 0)  # requires many iterations or many badPaths, always has few changes and low (but non-zero) error
  #cost = cp.norm1(x - nodeLabelsHotEnc) + 0.1 * cp.sum(G @ x - h)              # too many changed variables (so that cost of desired is as low as possible) but zero error
  #prob = cp.Problem(cp.Minimize(cost), [A @ x == b])
  #prob.solve(solver=cp.MOSEK, mosek_params={"MSK_DPAR_MIO_MAX_TIME":90})

  # costG ub
  cost = cp.norm1(x - nodeLabelsHotEnc)
  prob = cp.Problem(cp.Minimize(cost), [A @ x == b])
  prob.solve(solver=cp.MOSEK, mosek_params={"MSK_DPAR_MIO_MAX_TIME":90})
  costG_ub = np.max(G @ x.value - h)
  rospy.loginfo(costG_ub)

  # costG lb
  cost = cp.norm1(x - nodeLabelsHotEnc) + 1 * cp.maximum(cp.max(G @ x - h), 0)
  prob = cp.Problem(cp.Minimize(cost), [A @ x == b])
  prob.solve(solver=cp.MOSEK, mosek_params={"MSK_DPAR_MIO_MAX_TIME":90})
  costG_lb = np.max(G @ x.value - h)
  rospy.loginfo(costG_lb)

  lb_newPolyLabels, lb_newGraph = getPolyLabelsAndGraph(x, graph, nodeList, areaCosts)

  # trade-off curve
  tradeoffs = []
  curve_dist = []
  curve_l1 = []
  curve_newgraph = []
  curve_newpolylabels = []
  for tradeoff in np.linspace(0,1,11):
    # solve with desired trade-off
    rospy.loginfo("tradeoff %f" % (costG_lb + (costG_ub - costG_lb) * tradeoff))
    cost = cp.norm1(x - nodeLabelsHotEnc) + 1 * cp.maximum(cp.max(G @ x - h), costG_lb + (costG_ub - costG_lb) * tradeoff)
    prob = cp.Problem(cp.Minimize(cost), [A @ x == b])
    result = prob.solve(solver=cp.MOSEK, mosek_params={"MSK_DPAR_MIO_MAX_TIME":90})
    if result == float('inf'):
      rospy.loginfo("failed")
      continue
    newPolyLabels, newGraph = getPolyLabelsAndGraph(x, graph, nodeList, areaCosts)
    # save
    curve_newgraph.append(newGraph)
    curve_newpolylabels.append(newPolyLabels)
    # save tradeoff
    tradeoffs.append(tradeoff)
    # get distance
    path = nx.shortest_path(newGraph, source=desiredPath[0], target=desiredPath[-1], weight="weight")
    curve_dist.append(pathDistance(desiredPath, path))
    # get L1norm
    curve_l1.append(np.linalg.norm(x.value - nodeLabelsHotEnc, 1))

  # plot curve
  plt.figure(1)
  plt.clf()

  plt.subplot(121)
  plt.plot(tradeoffs, curve_dist, 'r', label="distance")
  plt.plot(tradeoffs, curve_l1, 'b', label="L1")
  plt.legend()
  plt.xlabel("alpha")
  plt.ylabel("value")

  plt.subplot(122)
  plt.plot(curve_l1, curve_dist)
  plt.xlabel("L1")
  plt.ylabel("distance")

  #plt.draw()
  #plt.pause(0.001)
  plt.show()

  # best solution
  mindist = min(curve_dist)
  idx = np.where(curve_dist <= mindist + 3)[0][-1]
  if mindist == 0:
    return curve_newpolylabels[idx], curve_newgraph[idx]
  else:
    return lb_newPolyLabels, lb_newGraph


def optPolyLabelsInPathTradeoff3(graph, areaCosts, desiredPath, badPaths):
  # variable:     l = nodeLabelsHotEnc                                      # vector of *in-path* node area-labels using one-hot encoding (bool)
  # cost:         (l-nodeLabelsHotEnc)^2
  # constraints:  edgeCost_i = dist_i * ac_0 * l_0 + dist_i * ac_1 * l_1    # for all edges i
  #               sum_(j in B) edgeCost_j - sum_(j in S_i) edgeCost_j <= 0  # for all paths S_i
  #               l_i + l_(i+1) = 1                                         # each node can have only one 1

  auxgraph = graph.copy()

  # define variable
  nodeLabelsHotEnc = []
  nodeList = []
  for node in graph.nodes:
    if graph.nodes[node]["area"] == -1:
      continue
    # check if this node is in any of the paths
    inpath = False
    if node in desiredPath:
      inpath = True
    if not inpath:
      for badPath in badPaths:
        if node in badPath:
          inpath = True
          break
    # add variables
    if inpath:
      if graph.nodes[node]["area"] == 1:
        auxgraph.nodes[node]["varidx"] = len(nodeLabelsHotEnc)
        nodeLabelsHotEnc.append(1)
        nodeLabelsHotEnc.append(0)
        nodeList.append(node)
      elif graph.nodes[node]["area"] == 2:
        auxgraph.nodes[node]["varidx"] = len(nodeLabelsHotEnc)
        nodeLabelsHotEnc.append(0)
        nodeLabelsHotEnc.append(1)
        nodeList.append(node)
  nodeLabelsHotEnc = np.array(nodeLabelsHotEnc)

  # path cost constraints
  G = []
  h = []
  for path in badPaths:
    line = [0.0]*len(nodeLabelsHotEnc)
    # sum_(i in B) dist_i * ac_0 * l_0 + dist_i * ac_1 * l_1
    for i in range(len(desiredPath)-1):
      d = dist(graph.nodes[desiredPath[i]]["point"], graph.nodes[desiredPath[i+1]]["point"])
      if graph.nodes[desiredPath[i]]["portal"] == False:
        line[ auxgraph.nodes[desiredPath[i]]["varidx"]   ] += d * areaCosts[1]
        line[ auxgraph.nodes[desiredPath[i]]["varidx"]+1 ] += d * areaCosts[2]
      elif graph.nodes[desiredPath[i+1]]["portal"] == False:
        line[ auxgraph.nodes[desiredPath[i+1]]["varidx"]   ] += d * areaCosts[1]
        line[ auxgraph.nodes[desiredPath[i+1]]["varidx"]+1 ] += d * areaCosts[2]
    # - sum_(i in S) dist_i * ac_0 * l_0 + dist_i * ac_1 * l_1
    for i in range(len(path)-1):
      d = dist(graph.nodes[path[i]]["point"], graph.nodes[path[i+1]]["point"])
      if graph.nodes[path[i]]["portal"] == False:
        line[ auxgraph.nodes[path[i]]["varidx"]   ] -= d * areaCosts[1]
        line[ auxgraph.nodes[path[i]]["varidx"]+1 ] -= d * areaCosts[2]
      elif graph.nodes[path[i+1]]["portal"] == False:
        line[ auxgraph.nodes[path[i+1]]["varidx"]   ] -= d * areaCosts[1]
        line[ auxgraph.nodes[path[i+1]]["varidx"]+1 ] -= d * areaCosts[2]
    G.append(line)
    h.append(0)
  G = np.array(G)
  h = np.array(h)

  # single active label per node
  A = []
  b = []
  for i in range(len(nodeList)):
    line = [0]*len(nodeLabelsHotEnc)
    line[2*i  ] = 1
    line[2*i+1] = 1
    A.append(line)
    b.append(1)
  A = np.array(A)
  b = np.array(b)

  # weights (by area, and penalize not-on-path)
  weights = [1]*len(nodeLabelsHotEnc)
  if False:
    for i in range(len(desiredPath)):
      if graph.nodes[desiredPath[i]]["portal"] == False:
        if i == 0:
          d = dist(graph.nodes[desiredPath[i]]["point"], graph.nodes[desiredPath[i+1]]["point"])
          weights[ auxgraph.nodes[desiredPath[i]]["varidx"]   ] = d
          weights[ auxgraph.nodes[desiredPath[i]]["varidx"]+1 ] = d
        elif i == len(desiredPath)-1:
          d = dist(graph.nodes[desiredPath[i-1]]["point"], graph.nodes[desiredPath[i]]["point"])
          weights[ auxgraph.nodes[desiredPath[i]]["varidx"]   ] = d
          weights[ auxgraph.nodes[desiredPath[i]]["varidx"]+1 ] = d
        else:
          d1 = dist(graph.nodes[desiredPath[i-1]]["point"], graph.nodes[desiredPath[i]]["point"])
          d2 = dist(graph.nodes[desiredPath[i]]["point"], graph.nodes[desiredPath[i+1]]["point"])
          weights[ auxgraph.nodes[desiredPath[i]]["varidx"]   ] = d1 + d2
          weights[ auxgraph.nodes[desiredPath[i]]["varidx"]+1 ] = d1 + d2
    for path in badPaths:
      for i in range(len(path)):
        if graph.nodes[path[i]]["portal"] == False:
          if i == 0:
            d = dist(graph.nodes[path[i]]["point"], graph.nodes[path[i+1]]["point"])
            weights[ auxgraph.nodes[path[i]]["varidx"]   ] = d
            weights[ auxgraph.nodes[path[i]]["varidx"]+1 ] = d
          elif i == len(path)-1:
            d = dist(graph.nodes[path[i-1]]["point"], graph.nodes[path[i]]["point"])
            weights[ auxgraph.nodes[path[i]]["varidx"]   ] = d
            weights[ auxgraph.nodes[path[i]]["varidx"]+1 ] = d
          else:
            d1 = dist(graph.nodes[path[i-1]]["point"], graph.nodes[path[i]]["point"])
            d2 = dist(graph.nodes[path[i]]["point"], graph.nodes[path[i+1]]["point"])
            weights[ auxgraph.nodes[path[i]]["varidx"]   ] = d1 + d2
            weights[ auxgraph.nodes[path[i]]["varidx"]+1 ] = d1 + d2
  if False:
    for i in range(len(nodeList)):
      if nodeList[i] not in desiredPath[1:-1]:
        weights[2*i  ] *= 10
        weights[2*i+1] *= 10

  # solve with cvxpy (hard constraints version ... has problems when it is impossible that the desired path is shortest)
  #x = cp.Variable(len(nodeLabelsHotEnc), boolean=True)
  #cost = cp.sum_squares(x - np.array(nodeLabelsHotEnc))
  #prob = cp.Problem(cp.Minimize(cost), [G @ x <= h, A @ x == b])
  #prob.solve(solver=cp.MOSEK, verbose=True, mosek_params={"MSK_DPAR_MIO_MAX_TIME":300})

  # solve with cvxpy (soft constraints version)
  x = cp.Variable(len(nodeLabelsHotEnc), boolean=True)
  #cost = cp.norm1(x - nodeLabelsHotEnc) + 1 * cp.maximum(cp.max(G @ x - h), 0)  # requires many iterations or many badPaths, always has few changes and low (but non-zero) error
  #cost = cp.norm1(x - nodeLabelsHotEnc) + 0.1 * cp.sum(G @ x - h)              # too many changed variables (so that cost of desired is as low as possible) but zero error
  #prob = cp.Problem(cp.Minimize(cost), [A @ x == b])
  #prob.solve(solver=cp.MOSEK, mosek_params={"MSK_DPAR_MIO_MAX_TIME":90})

  # costL1 lb
  cost = cp.norm1(cp.multiply(x - nodeLabelsHotEnc, weights))
  prob = cp.Problem(cp.Minimize(cost), [A @ x == b])
  prob.solve(solver=cp.MOSEK, mosek_params={"MSK_DPAR_MIO_MAX_TIME":90})
  costL1_lb = np.linalg.norm(np.multiply(x.value - nodeLabelsHotEnc, weights), 1)

  # costL1 ub
  cost = cp.sum(G @ x - h)
  prob = cp.Problem(cp.Minimize(cost), [A @ x == b])
  prob.solve(solver=cp.MOSEK, mosek_params={"MSK_DPAR_MIO_MAX_TIME":90})
  costL1_ub = np.linalg.norm(np.multiply(x.value - nodeLabelsHotEnc, weights), 1)

  # trade-off curve
  tradeoffs = []
  curve_dist = []
  curve_l1 = []
  curve_newgraph = []
  curve_newpolylabels = []
  for tradeoff in np.linspace(0,1,51):
    # solve with desired trade-off
    cost = cp.sum(G @ x - h)
    prob = cp.Problem(cp.Minimize(cost), [A @ x == b, cp.norm1(cp.multiply(x - nodeLabelsHotEnc, weights)) <= costL1_lb + (costL1_ub - costL1_lb) * tradeoff])
    result = prob.solve(solver=cp.MOSEK, mosek_params={"MSK_DPAR_MIO_MAX_TIME":90})
    if result == float('inf'):
      rospy.loginfo("failed")
      continue
    # get new graph/labels
    newPolyLabels, newGraph = getPolyLabelsAndGraph(x, graph, nodeList, areaCosts)
    # save
    curve_newgraph.append(newGraph)
    curve_newpolylabels.append(newPolyLabels)
    # save tradeoff
    tradeoffs.append(tradeoff)
    # get distance
    path = nx.shortest_path(newGraph, source=desiredPath[0], target=desiredPath[-1], weight="weight")
    curve_dist.append(pathDistance(desiredPath, path))
    # get L1norm
    curve_l1.append(np.linalg.norm(x.value - nodeLabelsHotEnc, 1))

  # plot curve
  plt.figure(1)
  plt.clf()

  plt.subplot(121)
  plt.plot(tradeoffs, curve_dist, 'r', label="distance")
  plt.plot(tradeoffs, curve_l1, 'b', label="L1")
  plt.legend()
  plt.xlabel("alpha")
  plt.ylabel("distance")

  plt.subplot(122)
  plt.plot(curve_l1, curve_dist)
  plt.xlabel("L1")
  plt.ylabel("distance")

  #plt.draw()
  #plt.pause(0.001)
  plt.show()

  # best solution
  mindist = min(curve_dist)
  idx = np.where(curve_dist <= mindist+6)[0][0]
  return curve_newpolylabels[idx], curve_newgraph[idx]

def optPolyLabelsInPathWithL1Target(graph, areaCosts, desiredPath, badPaths, l1):
  # variable:     l = nodeLabelsHotEnc                                      # vector of *in-path* node area-labels using one-hot encoding (bool)
  # cost:         (l-nodeLabelsHotEnc)^2
  # constraints:  edgeCost_i = dist_i * ac_0 * l_0 + dist_i * ac_1 * l_1    # for all edges i
  #               sum_(j in B) edgeCost_j - sum_(j in S_i) edgeCost_j <= 0  # for all paths S_i
  #               l_i + l_(i+1) = 1                                         # each node can have only one 1

  auxgraph = graph.copy()

  # define variable
  nodeLabelsHotEnc = []
  nodeList = []
  for node in graph.nodes:
    if graph.nodes[node]["area"] == -1:
      continue
    # check if this node is in any of the paths
    inpath = False
    if node in desiredPath:
      inpath = True
    if not inpath:
      for badPath in badPaths:
        if node in badPath:
          inpath = True
          break
    # add variables
    if inpath:
      if graph.nodes[node]["area"] == 1:
        auxgraph.nodes[node]["varidx"] = len(nodeLabelsHotEnc)
        nodeLabelsHotEnc.append(1)
        nodeLabelsHotEnc.append(0)
        nodeList.append(node)
      elif graph.nodes[node]["area"] == 2:
        auxgraph.nodes[node]["varidx"] = len(nodeLabelsHotEnc)
        nodeLabelsHotEnc.append(0)
        nodeLabelsHotEnc.append(1)
        nodeList.append(node)
  nodeLabelsHotEnc = np.array(nodeLabelsHotEnc)

  # path cost constraints
  G = []
  h = []
  for path in badPaths:
    line = [0.0]*len(nodeLabelsHotEnc)
    # sum_(i in B) dist_i * ac_0 * l_0 + dist_i * ac_1 * l_1
    for i in range(len(desiredPath)-1):
      d = dist(graph.nodes[desiredPath[i]]["point"], graph.nodes[desiredPath[i+1]]["point"])
      if graph.nodes[desiredPath[i]]["portal"] == False:
        line[ auxgraph.nodes[desiredPath[i]]["varidx"]   ] += d * areaCosts[1]
        line[ auxgraph.nodes[desiredPath[i]]["varidx"]+1 ] += d * areaCosts[2]
      elif graph.nodes[desiredPath[i+1]]["portal"] == False:
        line[ auxgraph.nodes[desiredPath[i+1]]["varidx"]   ] += d * areaCosts[1]
        line[ auxgraph.nodes[desiredPath[i+1]]["varidx"]+1 ] += d * areaCosts[2]
    # - sum_(i in S) dist_i * ac_0 * l_0 + dist_i * ac_1 * l_1
    for i in range(len(path)-1):
      d = dist(graph.nodes[path[i]]["point"], graph.nodes[path[i+1]]["point"])
      if graph.nodes[path[i]]["portal"] == False:
        line[ auxgraph.nodes[path[i]]["varidx"]   ] -= d * areaCosts[1]
        line[ auxgraph.nodes[path[i]]["varidx"]+1 ] -= d * areaCosts[2]
      elif graph.nodes[path[i+1]]["portal"] == False:
        line[ auxgraph.nodes[path[i+1]]["varidx"]   ] -= d * areaCosts[1]
        line[ auxgraph.nodes[path[i+1]]["varidx"]+1 ] -= d * areaCosts[2]
    G.append(line)
    h.append(0)
  G = np.array(G)
  h = np.array(h)

  # single active label per node
  A = []
  b = []
  for i in range(len(nodeList)):
    line = [0]*len(nodeLabelsHotEnc)
    line[2*i  ] = 1
    line[2*i+1] = 1
    A.append(line)
    b.append(1)
  A = np.array(A)
  b = np.array(b)

  # weights (by area, and penalize not-on-path)
  weights = [1]*len(nodeLabelsHotEnc)
  if True:
    for i in range(len(desiredPath)):
      if graph.nodes[desiredPath[i]]["portal"] == False:
        if i == 0:
          d = dist(graph.nodes[desiredPath[i]]["point"], graph.nodes[desiredPath[i+1]]["point"])
          weights[ auxgraph.nodes[desiredPath[i]]["varidx"]   ] = d
          weights[ auxgraph.nodes[desiredPath[i]]["varidx"]+1 ] = d
        elif i == len(desiredPath)-1:
          d = dist(graph.nodes[desiredPath[i-1]]["point"], graph.nodes[desiredPath[i]]["point"])
          weights[ auxgraph.nodes[desiredPath[i]]["varidx"]   ] = d
          weights[ auxgraph.nodes[desiredPath[i]]["varidx"]+1 ] = d
        else:
          d1 = dist(graph.nodes[desiredPath[i-1]]["point"], graph.nodes[desiredPath[i]]["point"])
          d2 = dist(graph.nodes[desiredPath[i]]["point"], graph.nodes[desiredPath[i+1]]["point"])
          weights[ auxgraph.nodes[desiredPath[i]]["varidx"]   ] = d1 + d2
          weights[ auxgraph.nodes[desiredPath[i]]["varidx"]+1 ] = d1 + d2
    for path in badPaths:
      for i in range(len(path)):
        if graph.nodes[path[i]]["portal"] == False:
          if i == 0:
            d = dist(graph.nodes[path[i]]["point"], graph.nodes[path[i+1]]["point"])
            weights[ auxgraph.nodes[path[i]]["varidx"]   ] = d
            weights[ auxgraph.nodes[path[i]]["varidx"]+1 ] = d
          elif i == len(path)-1:
            d = dist(graph.nodes[path[i-1]]["point"], graph.nodes[path[i]]["point"])
            weights[ auxgraph.nodes[path[i]]["varidx"]   ] = d
            weights[ auxgraph.nodes[path[i]]["varidx"]+1 ] = d
          else:
            d1 = dist(graph.nodes[path[i-1]]["point"], graph.nodes[path[i]]["point"])
            d2 = dist(graph.nodes[path[i]]["point"], graph.nodes[path[i+1]]["point"])
            weights[ auxgraph.nodes[path[i]]["varidx"]   ] = d1 + d2
            weights[ auxgraph.nodes[path[i]]["varidx"]+1 ] = d1 + d2
  if False:
    for i in range(len(nodeList)):
      if nodeList[i] not in desiredPath[1:-1]:
        weights[2*i  ] *= 10
        weights[2*i+1] *= 10

  # solve with cvxpy (soft constraints version)
  x = cp.Variable(len(nodeLabelsHotEnc), boolean=True)
  cost = cp.sum(G @ x - h)
  prob = cp.Problem(cp.Minimize(cost), [A @ x == b, cp.norm1(cp.multiply(x - nodeLabelsHotEnc, weights)) <= l1])
  result = prob.solve(solver=cp.MOSEK, mosek_params={"MSK_DPAR_MIO_MAX_TIME":90})
  if result == float('inf'):
    rospy.loginfo("failed")
    newPolyLabels = [0]*len(graph.nodes)
    for node in graph:
      newPolyLabels[ graph.nodes[node]["id"] ] = graph.nodes[node]["area"]
    return newPolyLabels, graph.copy()
  # get new graph/labels
  return getPolyLabelsAndGraph(x, graph, nodeList, areaCosts)


def optPolyLabelsInPathTradeoff4(graph, areaCosts, desiredPath, verbose=True, diversity_method="kdp-brandao", num_alternatives=2, max_iter=5):
  # trade-off curve
  curve_dist = []
  curve_l1 = []
  curve_newgraph = []
  curve_newpolylabels = []
  for l1 in np.linspace(1, tradeoff_maxL1, 21):
    if verbose:
      rospy.loginfo("Computing best explanation with L1 <= %f ...." % l1)
    ok, x, G, it = computeExplanationISP(graph, desiredPath[0], desiredPath[-1], areaCosts, desiredPath, "polyLabelsInPathWithL1Target", diversity_method, num_alternatives, max_iter, verbose, acceptable_dist=0.0, args=l1)
    # save
    curve_newgraph.append(G)
    curve_newpolylabels.append(x)
    curve_l1.append(l1)
    # get distance
    path = nx.shortest_path(G, source=desiredPath[0], target=desiredPath[-1], weight="weight")
    curve_dist.append(pathDistance(desiredPath, path))
  return curve_newpolylabels, curve_newgraph, curve_dist, curve_l1


def optAreaLabels(graph, areaCosts, allowedAreaTypes, desiredPath, badPaths):
  # variable:     l = labelSwitchHotEnc                                     # vector of all labels' new assigments using one-hot encoding (bool)
  # cost:         (l-labelsSwitchHotEnc)^2
  # constraints:  edgeCost_i = sum_(j) dist_i * ac_j * l_(oldlabel_i,j)     # for all edges i
  #               sum_(j in B) edgeCost_j - sum_(j in S_i) edgeCost_j <= 0  # for all paths S_i
  #               sum_(j) l_ij = 1                                          # for all labels i

  # define variable
  labelSwitchHotEnc = np.identity(len(allowedAreaTypes)).flatten()

  # path cost constraints
  G = []
  h = []
  for path in badPaths:
    line = [0.0]*len(labelSwitchHotEnc)
    # sum_(i in B) sum_(j) dist_i * ac_j * l_(oldlabel_i,j)
    for i in range(len(desiredPath)-1):
      d = dist(graph.nodes[desiredPath[i]]["point"], graph.nodes[desiredPath[i+1]]["point"])
      area = graph[desiredPath[i]][desiredPath[i+1]]["area"]
      areaIdx = allowedAreaTypes.index(area)
      for j in range(len(allowedAreaTypes)):
        line[ areaIdx*len(allowedAreaTypes)+j ] += d * areaCosts[ allowedAreaTypes[j] ]
    # - sum_(i in S) sum_(j) dist_i * ac_j * l_(oldlabel_i,j)
    for i in range(len(path)-1):
      d = dist(graph.nodes[path[i]]["point"], graph.nodes[path[i+1]]["point"])
      area = graph[path[i]][path[i+1]]["area"]
      areaIdx = allowedAreaTypes.index(area)
      for j in range(len(allowedAreaTypes)):
        line[ areaIdx*len(allowedAreaTypes)+j ] -= d * areaCosts[ allowedAreaTypes[j] ]
    G.append(line)
    h.append(0)
  G = np.array(G)
  h = np.array(h)

  # single active newlabel per oldlabel
  A = []
  b = []
  for i in range(len(allowedAreaTypes)):
    line = [1]*len(labelSwitchHotEnc)
    A.append(line)
    b.append(1)
  A = np.array(A)
  b = np.array(b)

  # solve with cvxpy (soft constraints version)
  x = cp.Variable(len(labelSwitchHotEnc), boolean=True)
  #cost = cp.norm1(x - np.array(labelSwitchHotEnc)) + 1 * cp.maximum(cp.max(G @ x - h), 0)
  cost = cp.norm1(x - np.array(labelSwitchHotEnc)) + 0.00001 * cp.sum(G @ x - h)
  prob = cp.Problem(cp.Minimize(cost), [A @ x == b])
  prob.solve(solver=cp.MOSEK, mosek_params={"MSK_DPAR_MIO_MAX_TIME":90})
  sol = x.value

  # get new labels and graph
  newLabels = [0]*len(allowedAreaTypes)
  newGraph = graph.copy()
  for i in range(len(allowedAreaTypes)):
    for j in range(len(allowedAreaTypes)):
      if sol[i*len(allowedAreaTypes)+j]:
        newLabels[i] = allowedAreaTypes[j]
  for node in newGraph:
    oldarea = graph.nodes[node]["area"]
    if oldarea != -1:
      oldareaIdx = allowedAreaTypes.index(oldarea)
      newarea = newLabels[oldareaIdx]
      newGraph.nodes[node]["area"] = newarea
      newGraph.nodes[node]["cost"] = areaCosts[newarea]
  for edge in list(newGraph.edges):
    if newGraph.nodes[edge[0]]["cost"] != -1:
      cost = newGraph.nodes[edge[0]]["cost"]
      area = newGraph.nodes[edge[0]]["area"]
    else:
      cost = newGraph.nodes[edge[1]]["cost"]
      area = newGraph.nodes[edge[1]]["area"]
    newGraph[edge[0]][edge[1]]["area"] = area
    newGraph[edge[0]][edge[1]]["cost"] = cost
    newGraph[edge[0]][edge[1]]["weight"] = cost * dist(newGraph.nodes[edge[0]]["point"], newGraph.nodes[edge[1]]["point"])
  return newLabels, newGraph


def optAreaLabelsEnum(graph, areaCosts, allowedAreaTypes, desiredPath, badPaths):
  # this will enumerate all area assignments and choose the one where the shortest path is closest to the desired

  # fail for large number of area types (5 types is already 3000 combinations so ~1hr)
  if len(allowedAreaTypes) > 5:
    return allowedAreaTypes, graph.clone()

  # solve with enumeration
  assignments = list( product(range(len(allowedAreaTypes)), repeat=len(allowedAreaTypes)) )
  mindist = float('Inf')
  for assignment in assignments:
    # new graph based on this assignment
    agraph = graph.copy()
    for edge in list(agraph.edges):
      oldarea = agraph[edge[0]][edge[1]]["area"]
      oldareaIdx = allowedAreaTypes.index(oldarea)
      newarea = allowedAreaTypes[ assignment[oldareaIdx] ]
      newcost = areaCosts[newarea]
      if agraph.nodes[edge[0]]["area"] != -1:
        agraph.nodes[edge[0]]["area"] = newarea
        agraph.nodes[edge[0]]["cost"] = newcost
      if agraph.nodes[edge[1]]["area"] != -1:
        agraph.nodes[edge[1]]["area"] = newarea
        agraph.nodes[edge[1]]["cost"] = newcost
      agraph[edge[0]][edge[1]]["area"] = newarea
      agraph[edge[0]][edge[1]]["cost"] = newcost
      agraph[edge[0]][edge[1]]["weight"] = newcost * dist(agraph.nodes[edge[0]]["point"], agraph.nodes[edge[1]]["point"])
    # compute shortest path distance
    path = nx.shortest_path(agraph, source=desiredPath[0], target=desiredPath[-1], weight="weight")
    d = pathDistance(desiredPath, path)
    if d < mindist:
      mindist = d
      newGraph = agraph
      newLabels = list(assignment)
      for i in range(len(newLabels)):
        newLabels[i] = allowedAreaTypes[ newLabels[i] ]
  return newLabels, newGraph


### path functions

def pathDistance(path1, path2):
  edges1 = []
  for i in range(len(path1)-1):
    edges1.append( (path1[i], path1[i+1]) )
  edges2 = []
  for i in range(len(path2)-1):
    edges2.append( (path2[i], path2[i+1]) )
  #return 1 - ( len(set(edges1).intersection(edges2)) / len(set(edges1).union(edges2)) )
  return similaritymeasures.frechet_dist(np.array(path1), np.array(path2))
  #return similaritymeasures.pcm(np.array(path1), np.array(path2))
  #return similaritymeasures.dtw(np.array(path1), np.array(path2))[0]


def pathIsNew(path, otherpaths, threshold):
  if len(otherpaths) == 0:
    return True
  for p in otherpaths:
    if pathDistance(path, p) < threshold:
      return False
  return True


def kDiversePathsVoss(graph, start, goal, k):
  max_attempts = 10
  weight_multiplier = 10
  similarity_threshold = 0.5
  newgraph = graph.copy()
  paths = []
  queue = []
  queue.append( nx.shortest_path(newgraph, source=start, target=goal, weight="weight") )
  while len(queue) > 0 and len(paths) < k:
    nextpath = queue.pop(0)
    for i in range(max_attempts):
      n = random.randint(0,len(nextpath)-2)
      newgraph[nextpath[n]][nextpath[n+1]]["weight"] *= weight_multiplier
      newpath = nx.shortest_path(newgraph, source=start, target=goal, weight="weight")
      if newpath in paths:
        continue
      elif pathIsNew(newpath, paths, similarity_threshold):
        paths.append(newpath)
      else:
        queue.append(newpath)
  return paths


def kDiversePathsBrandao(graph, start, goal, k):
  radius = 1
  weight_multiplier = 2
  newgraph = graph.copy()
  paths = []
  while len(paths) < k:
    # current shortest path
    path = nx.shortest_path(newgraph, source=start, target=goal, weight="weight")
    if path not in paths:
      paths.append(path)
    # update graph with increased weights on shortest path
    newgraph2 = newgraph.copy()
    for node in path:
      ego = nx.ego_graph(newgraph, node, radius)
      for edge in list(ego.edges):
        newgraph2[edge[0]][edge[1]]["weight"]  = newgraph[edge[0]][edge[1]]["weight"] * weight_multiplier
    newgraph = newgraph2
  return paths


def nearestNode(graph, node, distances):
  if node in graph:
    return node
  else:
    pdb.set_trace()
    neighbor_distances = distances[node]
    return min(neighbor_distances, key=neighbor_distances.get)


def kDiversePathsRRT(graph, start, goal, k):
  distances = distanceMatrix(graph)
  rospy.loginfo("... finished computing distanceMatrix")
  paths = []
  for i in range(k):
    # run an RRT from start to goal
    rrt = nx.Graph()
    rrt.add_node(start)
    while goal not in rrt.nodes:
      q_rand = random.choice(list(graph.nodes()))
      q_near = nearestNode(rrt, q_rand, distances)
      rrt.add_edge(q_rand, q_near)
    paths.append( nx.shortest_path(rrt, source=start, target=goal, weight="weight") )
  return paths


### path explanations

def computeExplanationISP(graph, start, goal, area_costs, desired_path, problem_type, diversity_method, num_alternatives, max_iter, verbose=True, acceptable_dist=0.0, args=[]):

  if verbose:
    rospy.loginfo("Computing explanation type %s, using %d %s-diverse paths and %d iterations..." % (problem_type, num_alternatives, diversity_method, max_iter))

  # check shortest path
  path = nx.shortest_path(graph, source=start, target=goal, weight="weight")
  if path == desired_path:
    rospy.loginfo("Shortest path is already equal to desired. Nothing to explain.")
    return True, [], graph.copy(), 0

  # init
  desired_is_shortest = False
  newgraph = graph.copy()
  all_alternative_paths = []
  history_explanations = []
  history_explanation_distances = []

  # iterate
  for it in range(max_iter):

    # compute diverse alternative paths
    time1 = time.clock()
    if diversity_method == "ksp":
      alternative_paths = list(islice(nx.shortest_simple_paths(newgraph, start, goal, weight="weight"), num_alternatives))
    elif diversity_method == "edp":
      alternative_paths = list(nx.edge_disjoint_paths(newgraph, start, goal, cutoff=num_alternatives))
    elif diversity_method == "kdp-voss":
      alternative_paths = kDiversePathsVoss(newgraph, start, goal, num_alternatives)
    elif diversity_method == "kdp-brandao":
      alternative_paths = kDiversePathsBrandao(newgraph, start, goal, num_alternatives)
    elif diversity_method == "kdp-rrt":
      alternative_paths = kDiversePathsRRT(newgraph, start, goal, num_alternatives)

    #all_alternative_paths += alternative_paths
    for alternative_path in alternative_paths:
      if alternative_path not in all_alternative_paths:
        all_alternative_paths.append(alternative_path)

    # compute explanation
    time2 = time.clock()
    if problem_type == "areaCosts":
      x,newgraph = optAreaCosts(graph, area_costs, desired_path, all_alternative_paths)
    elif problem_type == "polyCosts":
      x,newgraph = optPolyCosts(graph, desired_path, all_alternative_paths)
    elif problem_type == "polyLabelsApproxFromCosts":
      x,newgraph = optPolyLabelsApproxFromCosts(graph, area_costs, [1,2], desired_path, all_alternative_paths)
    elif problem_type == "polyLabelsApproxFromCosts2":
      ok1, x1, opt_costs_graph = computeExplanationISP(graph, start, goal, area_costs, desired_path, "polyCosts", diversity_method, num_alternatives, max_iter, False)
      x, newgraph = optPolyLabelsApproxFromCosts2(graph, area_costs, [1,2], desired_path, all_alternative_paths, opt_costs_graph)
    elif problem_type == "polyLabels":
      x,newgraph = optPolyLabels(graph, area_costs, desired_path, all_alternative_paths)
    elif problem_type == "polyLabelsInPath":
      x,newgraph = optPolyLabelsInPath(graph, area_costs, desired_path, all_alternative_paths)
    elif problem_type == "polyLabelsInPathTradeoff":
      x,newgraph = optPolyLabelsInPathTradeoff(graph, area_costs, desired_path, all_alternative_paths)
    elif problem_type == "polyLabelsInPathTradeoff2":
      x,newgraph = optPolyLabelsInPathTradeoff2(graph, area_costs, desired_path, all_alternative_paths)
    elif problem_type == "polyLabelsInPathTradeoff3":
      x,newgraph = optPolyLabelsInPathTradeoff3(graph, area_costs, desired_path, all_alternative_paths)
    elif problem_type == "polyLabelsInPathWithL1Target":
      x,newgraph = optPolyLabelsInPathWithL1Target(graph, area_costs, desired_path, all_alternative_paths, args)
    elif problem_type == "areaLabels":
      x,newgraph = optAreaLabels(graph, area_costs, [1,2], desired_path, all_alternative_paths)
    elif problem_type == "areaLabelsEnum":
      x,newgraph = optAreaLabelsEnum(graph, area_costs, [1,2], desired_path, all_alternative_paths)
    time3 = time.clock()

    # compute new shortest path
    path = nx.shortest_path(newgraph, source=start, target=goal, weight="weight")

    # save
    history_explanations.append( newgraph.copy() )
    history_explanation_distances.append( pathDistance(desired_path, path) )
    if verbose:
      rospy.loginfo("  iteration %d took %f + %f secs. Similarity to desired path = %f" % (it, time2-time1, time3-time2, pathDistance(desired_path, path)))

    # check that desired path is the shortest in new graph
    if pathDistance(desired_path, path) <= acceptable_dist:
      desired_is_shortest = True
      break

  # pick best explanation (since performance usually fluctuates as we add more alternative paths)
  best_explanation = np.argmin(history_explanation_distances)
  newgraph = history_explanations[best_explanation]
  if verbose:
    rospy.loginfo("...finished %s in %d iterations. Best explanation leads to a shortest-desired path distance = %f" % (problem_type, it+1, history_explanation_distances[best_explanation]))

  return desired_is_shortest, x, newgraph, it+1


def benchmarkExplanationISP(graph, start, goal, area_costs, desired_path, problem_type, verbose, acceptable_dist):
  # conclusions so far:
  # 1.often no need for more than one path (no need for diverse paths), it's better to just get the shortest path
  #   then obtain a graph explanation, and use the shortest path of that graph as the next constraint (so we focus on the
  #   edges that are keeping the shortest path to be the desired). We would need to have extremely high number of
  #   alternative paths on diversity fuction for iterations not to be necessary. Iterating explanation and shortest-paths more effective.
  #   However, kdp-brandao with a few paths is useful in more challenging problems.
  #   TLDR: just use ksp-1 or kdp-brandao-2/5
  # 2.edp is a bad choice of alternative paths (because shortest path not in list?)
  # 3.need a few iterations of explanation-computation and constraint-addition until convergence of shortest path to desired path
  # 4.approximations to the polyLabels problem (such as computing labels from costs) are poor, better to just use full 
  #   optimization of labels along desired and alternative paths (optPolyLabelsInPath)

  rospy.loginfo("--------------------------------------------------")
  rospy.loginfo("Benchmarking %s..." % problem_type)

  ### inverse optimization methods
  if problem_type in ["invLP", "invMILP", "invMILPapprox", "astarPolyLabels"]:

    # compute explanation
    time1 = time.clock()
    if problem_type == "invLP":
      newweights, newgraph = optInvLP(graph, desired_path)
    if problem_type == "invMILP":
      newweights, newgraph = optInvMILP(graph, desired_path, area_costs, [1,2], exact=True, verbose=verbose)
    if problem_type == "invMILPapprox":
      newweights, newgraph = optInvMILP(graph, desired_path, area_costs, [1,2], exact=False, verbose=verbose)
    if problem_type == "astarPolyLabels":
      newweights, newgraph = astarPolyLabels(graph, desired_path, area_costs, [1,2], verbose=verbose)
    time2 = time.clock()

    # compute shortest path
    path = nx.shortest_path(newgraph, source=start, target=goal, weight="weight")

    # compute l1norm and number of changed variables
    x1 = []
    x2 = []
    if problem_type == "invLP":
      for edge in list(graph.edges):
        x1.append(graph[edge[0]][edge[1]]["cost"])
        x2.append(newgraph[edge[0]][edge[1]]["cost"])
    if problem_type in ["invMILP", "invMILPapprox", "astarPolyLabels"]:
      for edge in list(graph.edges):
        x1.append(graph[edge[0]][edge[1]]["area"])
        x2.append(newgraph[edge[0]][edge[1]]["area"])
    x1 = np.array(x1)
    x2 = np.array(x2)
    l1norm = np.linalg.norm(x2 - x1, 1)
    changed = np.sum( np.abs(x2 - x1) > 0.01 )

    # log results
    result = {}
    result["problem_type"] = problem_type
    result["diversity_method"] = ""
    result["num_alternatives"] = ""
    result["max_iter"] = ""
    result["num_iter"] = ""
    result["graph"] = newgraph
    result["l1_norm"] = l1norm
    result["changed_vars"] = changed
    result["changed_vars_per"] = 100 * changed / len(x2)
    result["dist"] = pathDistance(desired_path, path)
    result["time"] = time2 - time1
    results = []
    results.append(result)
    rospy.loginfo("Results:\n" + str(tabulate(results, headers="keys")))
    return results

  ### goodpath-badpaths methods
  results = []
  for diversity_method in ["ksp", "kdp-brandao"]:
    for num_alternatives in [1, 2, 5, 10]:
      for max_iter in [5,10]:

        # compute explanation
        time1 = time.clock()
        if problem_type == "polyLabelsInPathTradeoff4":
          vx, vG, vdist, vl1 = optPolyLabelsInPathTradeoff4(graph, area_costs, desired_path, verbose, diversity_method, num_alternatives, max_iter)
          newgraph = vG[ np.where(vdist <= min(vdist))[0][0] ]
          num_iter = 1
        else:
          ok, x, newgraph, num_iter = computeExplanationISP(graph, start, goal, area_costs, desired_path, problem_type, diversity_method, num_alternatives, max_iter, verbose, acceptable_dist)
        time2 = time.clock()

        # compute shortest path
        path = nx.shortest_path(newgraph, source=start, target=goal, weight="weight")

        # compute l1norm and number of changed variables
        if problem_type == "areaCosts":
          x1 = np.array(area_costs)
          x2 = np.array(x)
        elif problem_type == "polyCosts":
          x1 = []
          x2 = []
          for edge in list(graph.edges):
            x1.append(graph[edge[0]][edge[1]]["cost"])
            x2.append(newgraph[edge[0]][edge[1]]["cost"])
          x1 = np.array(x1)
          x2 = np.array(x2)
        else: # polyLabels and approximations
          x1 = []
          x2 = []
          for edge in list(graph.edges):
            x1.append(graph[edge[0]][edge[1]]["area"])
            x2.append(newgraph[edge[0]][edge[1]]["area"])
          x1 = np.array(x1)
          x2 = np.array(x2)
        l1norm = np.linalg.norm(x2 - x1, 1)
        changed = np.sum( np.abs(x2 - x1) > 0.01 )

        # log results
        result = {}
        result["problem_type"] = problem_type
        result["diversity_method"] = diversity_method
        result["num_alternatives"] = num_alternatives
        result["max_iter"] = max_iter
        result["num_iter"] = num_iter
        result["graph"] = newgraph
        result["l1_norm"] = l1norm
        result["changed_vars"] = changed
        result["changed_vars_per"] = 100 * changed / len(x2)
        result["dist"] = pathDistance(desired_path, path)
        result["time"] = time2 - time1
        results.append(result)

  rospy.loginfo("Results:\n" + str(tabulate(results, headers="keys")))
  return results

### callback

tradeoff_visualized_index = None
tradeoff_maxL1 = None
contrastive_waypoint = None

def callback(config, level):
  global tradeoff_visualized_index, tradeoff_maxL1
  rospy.loginfo("""Reconfigure Request: {tradeoff_visualized_index}, {tradeoff_maxL1}""".format(**config))
  tradeoff_visualized_index = config.tradeoff_visualized_index
  tradeoff_maxL1 = config.tradeoff_maxL1
  return config

def callbackContrastiveWaypoint(data):
  global contrastive_waypoint
  #rospy.loginfo("Received contrastive waypoint")
  contrastive_waypoint = data


### visualization

def mypause(interval):
  backend = plt.rcParams['backend']
  if backend in matplotlib.rcsetup.interactive_bk:
    figManager = matplotlib._pylab_helpers.Gcf.get_active()
    if figManager is not None:
      canvas = figManager.canvas
      if canvas.figure.stale:
        canvas.draw()
      canvas.start_event_loop(interval)
      return

### main

if __name__ == "__main__":

  debug = False

  # ros init
  rospy.init_node('recast_explanations')

  pubGraph = rospy.Publisher('graph', MarkerArray, queue_size=10)
  pubGraphCosts = rospy.Publisher('graph_costs', MarkerArray, queue_size=10)
  pubPath = rospy.Publisher('graph_path', Marker, queue_size=10)
  pubPathDesired = rospy.Publisher('graph_path_desired', Marker, queue_size=10)

  rospy.Subscriber("/recast_explanations_interface/contrastive_position", Point, callbackContrastiveWaypoint)

  pubDiv1 = rospy.Publisher('graph_diversity1', MarkerArray, queue_size=10)
  pubDiv2 = rospy.Publisher('graph_diversity2', MarkerArray, queue_size=10)
  pubDiv3 = rospy.Publisher('graph_diversity3', MarkerArray, queue_size=10)
  pubDiv4 = rospy.Publisher('graph_diversity4', MarkerArray, queue_size=10)
  pubDiv5 = rospy.Publisher('graph_diversity5', MarkerArray, queue_size=10)

  pubAreaCostsPath =   rospy.Publisher('expl_area_costs_path',   Marker,      queue_size=10)
  pubAreaCostsGraph =  rospy.Publisher('expl_area_costs_graph',  MarkerArray, queue_size=10)
  pubPolyCostsPath =   rospy.Publisher('expl_poly_costs_path',   Marker,      queue_size=10)
  pubPolyCostsGraph =  rospy.Publisher('expl_poly_costs_graph',  MarkerArray, queue_size=10)
  pubPolyLabelsPath =  rospy.Publisher('expl_poly_labels_path',  Marker,      queue_size=10)
  pubPolyLabelsGraph = rospy.Publisher('expl_poly_labels_graph', MarkerArray, queue_size=10)
  pubPolyLabels4Path = rospy.Publisher('expl_poly_labels4_path', Marker,      queue_size=10)
  pubPolyLabels4Graph =rospy.Publisher('expl_poly_labels4_graph',MarkerArray, queue_size=10)
  pubAreaLabelsPath =  rospy.Publisher('expl_area_labels_path',  Marker,      queue_size=10)
  pubAreaLabelsGraph = rospy.Publisher('expl_area_labels_graph', MarkerArray, queue_size=10)
  pubTradeoffPolyLabelsPath =  rospy.Publisher('expl_tradeoff_poly_labels_path',  Marker,      queue_size=10)
  pubTradeoffPolyLabelsGraph = rospy.Publisher('expl_tradeoff_poly_labels_graph', MarkerArray, queue_size=10)
  pubInvLpPath =       rospy.Publisher('expl_inv_lp_path',       Marker,      queue_size=10)
  pubInvLpGraph =      rospy.Publisher('expl_inv_lp_graph',      MarkerArray, queue_size=10)
  pubInvMiLpPath =     rospy.Publisher('expl_inv_milp_path',     Marker,      queue_size=10)
  pubInvMiLpGraph =    rospy.Publisher('expl_inv_milp_graph',    MarkerArray, queue_size=10)

  srv = Server(ExplanationsConfig, callback)

  rospy.loginfo('Waiting for recast_ros...')
  rospy.wait_for_service('/recast_node/plan_path')
  rospy.wait_for_service('/recast_node/project_point')

  rosgraph = None
  areaCosts = None
  pstart = None
  pgoal = None
  pwaypoint = None
  old_rosgraph = None
  old_areaCosts = None
  old_pstart = None
  old_pgoal = None
  old_pwaypoint = None

  # prepare plots
  show_tradeoff = False
  if show_tradeoff:
    plt.ion()
    fig = plt.figure()
    plt.xlabel("L1")
    plt.ylabel("distance")
    plt.show(block=False)

  # loop
  rate = rospy.Rate(1.0) # hz
  while not rospy.is_shutdown():
    rate.sleep()

    rospy.loginfo('=======================================================')

    # keep old data
    if rosgraph is not None:
      old_rosgraph = rosgraph
    if areaCosts is not None:
      old_areaCosts = areaCosts.copy()
    if pstart is not None:
      old_pstart = pstart
    if pgoal is not None:
      old_pgoal = pgoal
    if pwaypoint is not None:
      old_pwaypoint = pwaypoint

    # get area costs
    rospy.loginfo('Getting area costs...')
    areaCosts = [0]
    for i in range(1,20):
      areaCosts.append( rospy.get_param('/recast_node/TERRAIN_TYPE' + str(i) + '_COST') )
    if debug:
      rospy.loginfo('area costs = ' + str(areaCosts))

    # get graph
    rospy.loginfo('Getting graph...')
    rosgraph = rospy.wait_for_message('/recast_node/graph', RecastGraph)
    if debug:
      rospy.loginfo('nodes = ' + str(len(rosgraph.nodes)) + ' portals = ' + str(len(rosgraph.portals)))

    # networkx graph
    Gdirect = nx.Graph()
    G = nx.Graph()
    N = {}
    for i in range(0, len(rosgraph.nodes), 2):
      # get edge
      k1, data1 = addNode(rosgraph.nodes[i  ], areaCosts, N)
      k2, data2 = addNode(rosgraph.nodes[i+1], areaCosts, N)
      # check whether already connected (not to have repeated edges)
      if k1 in Gdirect and k2 in Gdirect[k1]:
        continue
      else:
        Gdirect.add_edge(k1,k2)
      # get edge midpoint (portal point)
      km, datam = addPortal(rosgraph.portals[int(i/2)], N)
      # add two edges (1 to midpoint, midpoint to 2)
      cost1 = data1["cost"] * dist(data1["point"], datam["point"])
      cost2 = data2["cost"] * dist(datam["point"], data2["point"])
      G.add_edge(k1, km, weight=cost1, area=data1["area"], cost=data1["cost"])
      G.add_edge(km, k2, weight=cost2, area=data2["area"], cost=data2["cost"])
      copyDict(datam, G.nodes[km])
      copyDict(data1, G.nodes[k1])
      copyDict(data2, G.nodes[k2])
    rospy.loginfo('Finished.')

    # get path
    rospy.loginfo('Getting path...')
    rospath = rospy.wait_for_message('/recast_node/recast_path_lines', Marker)
    rospath_centers = []
    #for i in range(0, len(rospath.points), 2) + [len(rospath.points)-1]:
    for i in list(range(0, len(rospath.points), 2)) + [len(rospath.points)-1]:
      proj = getProj(rospath.points[i])
      rospath_centers.append(proj.projected_polygon_center)

    # closest nodes to start/goal
    pstart = getKey(rospath_centers[0])
    pgoal = getKey(rospath_centers[-1])

    # path on graph
    rospy.loginfo('Solving shortest path on our local graph...')
    gpath = nx.shortest_path(G, source=pstart, target=pgoal, weight="weight")

    # path stats
    if debug:
      for k in gpath:
        rospy.loginfo('area = ' + str(N[k]["area"]) + ' cost = ' + str(N[k]["cost"]))
      totalcost = 0
      for i in range(len(gpath)-1):
        cost = G[gpath[i]][gpath[i+1]]['weight']
        rospy.loginfo(str(cost))
        totalcost += cost
      rospy.loginfo('total cost = ' + str(totalcost))

    # desired path on graph (v1)
    #rospy.loginfo('Solving shortest-hop ("desired") path on our local graph...')
    #gpath_desired = nx.shortest_path(G, source=pstart, target=pgoal)

    # desired path on graph (v2)
    rospy.loginfo('Computing desired path from waypoint...')
    Gdesired = G.copy()
    for edge in Gdesired.edges:
      area = 1
      Gdesired[edge[0]][edge[1]]["area"] = area
      Gdesired[edge[0]][edge[1]]["cost"] = areaCosts[area]
      Gdesired[edge[0]][edge[1]]["weight"] = areaCosts[area] * dist(Gdesired.nodes[edge[0]]["point"], Gdesired.nodes[edge[1]]["point"])
    if contrastive_waypoint == None:
      # shortest path on area1
      gpath_desired = nx.shortest_path(Gdesired, source=pstart, target=pgoal, weight="weight")
    else:
      # get closest node
      proj = getProj(contrastive_waypoint)
      pwaypoint = getKey(proj.projected_polygon_center)
      # shortest path on area1, through waypoint
      gpath_desired = nx.shortest_path(Gdesired, source=pstart, target=pwaypoint, weight="weight")[:-1]
      gpath_desired += nx.shortest_path(Gdesired, source=pwaypoint, target=pgoal, weight="weight")

    # compare costs
    cost_path = getCost(G, gpath)
    cost_path_desired = getCost(G, gpath_desired)
    rospy.loginfo('Cost of desired path is %f >= %f' % (cost_path_desired, cost_path))

    # LP shortest path
    if False:
      rospy.loginfo('Computing shortest path from LP...')
      lp_path = findShortestPathLP(G, pstart, pgoal)

    # visualize graph and paths
    if pubGraph.get_num_connections() > 0:
      rospy.loginfo('Visualizing our graph...')
      #pubGraph.publish( graphToMarkerArrayByArea(G, 0.2, list(range(-1,len(areaCosts)))) )
      pubGraph.publish( graphToMarkerArray(G, 0.2) )
    if pubGraphCosts.get_num_connections() > 0:
      rospy.loginfo('Visualizing our graph costs...')
      pubGraphCosts.publish( graphToMarkerArrayByCost(G, 0.1) )

    if pubPath.get_num_connections() > 0:
      rospy.loginfo('Visualizing our shortest path...')
      pubPath.publish( pathToMarker(G, gpath, 0, [1,0.5,0,1], 1) )

    if pubPathDesired.get_num_connections() > 0:
      rospy.loginfo('Visualizing our desired path...')
      pubPathDesired.publish( pathToMarker(G, gpath_desired, 0, [0,1,0,1], 1) )

    # visualize diversity methods
    if True:
      k = 10
      if pubDiv1.get_num_connections() > 0:
        rospy.loginfo('Visualizing diverse path function 1 (ksp)...')
        div1 = list(islice(nx.shortest_simple_paths(G, pstart, pgoal, weight="weight"), k))
        pubDiv1.publish( pathsToMarkerArray(G, div1, 0.9) )

      if pubDiv2.get_num_connections() > 0:
        rospy.loginfo('Visualizing diverse path function 2 (edp)...')
        div2 = list(nx.edge_disjoint_paths(G, pstart, pgoal, cutoff=k))
        pubDiv2.publish( pathsToMarkerArray(G, div2, 0.9) )

      #if pubDiv3.get_num_connections() > 0:
      #  rospy.loginfo('Visualizing diverse path function 3 (kdp-voss)...')
      #  div3 = kDiversePathsVoss(G, pstart, pgoal, k)
      #  pubDiv3.publish( pathsToMarkerArray(G, div3, 0.9) )

      if pubDiv4.get_num_connections() > 0:
        rospy.loginfo('Visualizing diverse path function 4 (kdp-brandao)...')
        div4 = kDiversePathsBrandao(G, pstart, pgoal, k)
        pubDiv4.publish( pathsToMarkerArray(G, div4, 0.9) )

      #if pubDiv5.get_num_connections() > 0:
      #  rospy.loginfo('Visualizing diverse path function 5 (rrt)...')
      #  div5 = kDiversePathsRRT(G, pstart, pgoal, k)
      #  pubDiv5.publish( pathsToMarkerArray(G, div5, 0.9) )

    # compute & visualize explanations by inverse shortest paths
    if pubAreaCostsGraph.get_num_connections() > 0 or pubAreaCostsPath.get_num_connections() > 0:
      rospy.loginfo("Computing explanation based on areaCosts...")
      ok1, x1, G1, it1 = computeExplanationISP(G, pstart, pgoal, areaCosts, gpath_desired, "areaCosts", "kdp-brandao", 2, 10, verbose=False, acceptable_dist=3.0)
      xpath1 = nx.shortest_path(G1, source=pstart, target=pgoal, weight="weight")
      # visualize
      pubAreaCostsPath.publish( pathToMarker(G1, xpath1, 0, [1,0,0,1], 0.9) )
      pubAreaCostsGraph.publish( graphToMarkerArrayByCost(G1, 0.2) )

    if pubPolyCostsPath.get_num_connections() > 0 or pubPolyCostsGraph.get_num_connections() > 0:
      rospy.loginfo("Computing explanation based on polyCosts...")
      ok2, x2, G2, it2 = computeExplanationISP(G, pstart, pgoal, areaCosts, gpath_desired, "polyCosts", "kdp-brandao", 2, 10, verbose=False, acceptable_dist=3.0)
      xpath2 = nx.shortest_path(G2, source=pstart, target=pgoal, weight="weight")
      # visualize
      pubPolyCostsPath.publish( pathToMarker(G2, xpath2, 0, [1,0,0,1], 0.9) )
      pubPolyCostsGraph.publish( graphToMarkerArrayByCost(G2, 0.2) )

    if pubInvLpPath.get_num_connections() > 0 or pubInvLpGraph.get_num_connections() > 0:
      rospy.loginfo('Computing explanation based on inverse LP...')
      invlp_weights, invlp_graph = optInvLP(G, gpath_desired)
      invlp_xpath = nx.shortest_path(invlp_graph, source=pstart, target=pgoal, weight="weight")
      # visualize
      pubInvLpPath.publish( pathToMarker(invlp_graph, invlp_xpath, 0, [1,0,0,1], 0.9) )
      pubInvLpGraph.publish( graphToMarkerArrayByCost(invlp_graph, 0.2) )

    if pubInvMiLpPath.get_num_connections() > 0 or pubInvMiLpGraph.get_num_connections() > 0:
      rospy.loginfo('Computing explanation based on inverse MILP...')
      invmilp_weights, invmilp_graph = optInvMILP(G, gpath_desired, areaCosts, [1,2])
      invmilp_xpath = nx.shortest_path(invmilp_graph, source=pstart, target=pgoal, weight="weight")
      # visualize
      pubInvMiLpPath.publish( pathToMarker(invmilp_graph, invmilp_xpath, 0, [1,0,0,1], 0.9) )
      pubInvMiLpGraph.publish( graphToMarkerArray(invmilp_graph, 0.2) )

    if pubPolyLabelsPath.get_num_connections() > 0 or pubPolyLabelsGraph.get_num_connections() > 0:
      rospy.loginfo("Computing explanation based on polyLabelsInPath...")
      ok3, x3, G3, it3 = computeExplanationISP(G, pstart, pgoal, areaCosts, gpath_desired, "polyLabelsInPath", "kdp-brandao", 2, 10, verbose=True, acceptable_dist=1.0)
      xpath3 = nx.shortest_path(G3, source=pstart, target=pgoal, weight="weight")
      # visualize
      G3changes = getGraphChangesForVis(G, G3)
      pubPolyLabelsPath.publish( pathToMarker(G3, xpath3, 0, [1,0,0,1], 0.9) )
      pubPolyLabelsGraph.publish( graphToMarkerArray(G3changes, 0.2) )

    if pubPolyLabels4Path.get_num_connections() > 0 or pubPolyLabels4Graph.get_num_connections() > 0:
      rospy.loginfo("Computing explanation based on polyLabelsInPathTradeoff4...")
      xto_vec, Gto_vec, distto_vec, l1to_vec = optPolyLabelsInPathTradeoff4(G, areaCosts, gpath_desired, verbose=False)
      G3 = Gto_vec[ np.where(distto_vec <= min(distto_vec))[0][0] ]
      xpath3 = nx.shortest_path(G3, source=pstart, target=pgoal, weight="weight")
      # visualize
      G3changes = getGraphChangesForVis(G, G3)
      pubPolyLabels4Path.publish( pathToMarker(G3, xpath3, 0, [1,0,0,1], 0.9) )
      pubPolyLabels4Graph.publish( graphToMarkerArray(G3changes, 0.2) )

    if pubAreaLabelsPath.get_num_connections() > 0 or pubAreaLabelsGraph.get_num_connections() > 0:
      rospy.loginfo("Computing explanation based on areaLabelsEnum...")
      ok4, x4, G4, it4 = computeExplanationISP(G, pstart, pgoal, areaCosts, gpath_desired, "areaLabelsEnum", "kdp-brandao", 2, 10, verbose=False, acceptable_dist=3.0)
      xpath4 = nx.shortest_path(G4, source=pstart, target=pgoal, weight="weight")
      # visualize
      G4changes = getGraphChangesForVis(G, G4)
      pubAreaLabelsPath.publish( pathToMarker(G4, xpath4, 0, [1,0,0,1], 0.9) )
      pubAreaLabelsGraph.publish( graphToMarkerArray(G4changes, 0.2) )

    # trade-off curve
    if show_tradeoff:
      if old_rosgraph != rosgraph or old_areaCosts != areaCosts or old_pstart != pstart or old_pgoal != pgoal or old_pwaypoint != pwaypoint:
        # compute
        rospy.loginfo("Computing explanation trade-offs based on polyLabelsInPath...")
        to_x_vec, to_G_vec, to_dist_vec, to_l1_vec = optPolyLabelsInPathTradeoff4(G, areaCosts, gpath_desired, verbose=True)
        to_path_marker_vec = []
        to_graph_marker_vec = []
        for to_G in to_G_vec:
          to_xpath = nx.shortest_path(to_G, source=pstart, target=pgoal, weight="weight")
          to_Gchanges = getGraphChangesForVis(G, to_G)
          to_path_marker_vec.append( pathToMarker(to_G, to_xpath, 0, [1,0,0,1], 0.9) )
          to_graph_marker_vec.append( graphToMarkerArray(to_Gchanges, 0.2) )
      # pick index from dynamic reconfigure
      idx = tradeoff_visualized_index
      # show curve
      rospy.loginfo("Showing trade-off curve...")
      plt.gca().clear()
      plt.plot(to_l1_vec, to_dist_vec, marker='.')
      plt.plot(to_l1_vec[idx], to_dist_vec[idx], 'r', marker='o')
      mypause(0.001)
      #plt.xlabel("L1")
      #plt.ylabel("distance")
      #fig.canvas.draw()
      #fig.canvas.flush_events()
      rospy.loginfo("Visualizing selected point...")
      # visualize
      if pubTradeoffPolyLabelsPath.get_num_connections() > 0 or pubTradeoffPolyLabelsGraph.get_num_connections() > 0:
        to_path_marker_vec[idx].header.stamp = rospy.Time.now()
        for marker in to_graph_marker_vec[idx].markers:
          marker.header.stamp = rospy.Time.now()
        pubTradeoffPolyLabelsPath.publish( to_path_marker_vec[idx] )
        pubTradeoffPolyLabelsGraph.publish( to_graph_marker_vec[idx] )

    # Running bencharks
    if False:
      rospy.loginfo("Running benchmarks for each problem type...")
      benchmarkExplanationISP(G, pstart, pgoal, areaCosts, gpath_desired, "areaCosts", verbose=False, acceptable_dist=5.0)
      benchmarkExplanationISP(G, pstart, pgoal, areaCosts, gpath_desired, "polyCosts", verbose=False, acceptable_dist=5.0)
      benchmarkExplanationISP(G, pstart, pgoal, areaCosts, gpath_desired, "polyLabelsInPath", verbose=False, acceptable_dist=5.0)
      benchmarkExplanationISP(G, pstart, pgoal, areaCosts, gpath_desired, "areaLabelsEnum", verbose=False, acceptable_dist=5.0)

    if False:
      rospy.loginfo("Running benchmark [polyLabelsInPath vs areaLabels] ...")
      benchmarkExplanationISP(G, pstart, pgoal, areaCosts, gpath_desired, "areaLabelsEnum", verbose=False, acceptable_dist=5.0)
      benchmarkExplanationISP(G, pstart, pgoal, areaCosts, gpath_desired, "areaLabels", verbose=False, acceptable_dist=5.0)
      benchmarkExplanationISP(G, pstart, pgoal, areaCosts, gpath_desired, "polyLabelsInPath", verbose=False, acceptable_dist=5.0)
      # note: - polyLabels expected to be better (provide better explanations i.e. distance-to-desired-path) when desired path is not shortest
      #       - also better in terms of number of nodes/edges that changed label ("changed_vars" column)
      #       - areaLabels expected to be faster for low number of area types
      #       - need way to input desired path

    if False:
      rospy.loginfo("Running benchmark [polyLabelsInPath VS polyLabels] for maximum number of iterations...")
      benchmarkExplanationISP(G, pstart, pgoal, areaCosts, gpath_desired, "areaLabelsEnum", verbose=False, acceptable_dist=0.0)
      benchmarkExplanationISP(G, pstart, pgoal, areaCosts, gpath_desired, "polyLabelsInPath", verbose=False, acceptable_dist=0.0)
      benchmarkExplanationISP(G, pstart, pgoal, areaCosts, gpath_desired, "polyLabels", verbose=False, acceptable_dist=0.0)

    if False:
      rospy.loginfo("Running benchmark for polyLabels [astar VS invopt]...")
      results2 = benchmarkExplanationISP(G, pstart, pgoal, areaCosts, gpath_desired, "invMILP", verbose=False, acceptable_dist=0.0)
      results2+= benchmarkExplanationISP(G, pstart, pgoal, areaCosts, gpath_desired, "astarPolyLabels", verbose=False, acceptable_dist=0.0)
      rospy.loginfo("Benchmark for polyLabels [astar VS invopt]:")
      rospy.loginfo("\n"+str(tabulate(results2, headers="keys")))

    if False:
      rospy.loginfo("Running benchmark for polyCosts...")
      results1 = []
      results1+= benchmarkExplanationISP(G, pstart, pgoal, areaCosts, gpath_desired, "polyCosts", verbose=False, acceptable_dist=0.0)
      results1+= benchmarkExplanationISP(G, pstart, pgoal, areaCosts, gpath_desired, "invLP", verbose=False, acceptable_dist=0.0)
      rospy.loginfo("Running benchmark for polyLabels...")
      results2 = []
      results2+= benchmarkExplanationISP(G, pstart, pgoal, areaCosts, gpath_desired, "astarPolyLabels", verbose=False, acceptable_dist=0.0)
      results2+= benchmarkExplanationISP(G, pstart, pgoal, areaCosts, gpath_desired, "areaLabelsEnum", verbose=False, acceptable_dist=0.0)
      results2+= benchmarkExplanationISP(G, pstart, pgoal, areaCosts, gpath_desired, "polyLabelsInPath", verbose=False, acceptable_dist=0.0)
      results2+= benchmarkExplanationISP(G, pstart, pgoal, areaCosts, gpath_desired, "polyLabelsInPathTradeoff4", verbose=False, acceptable_dist=0.0)
      results2+= benchmarkExplanationISP(G, pstart, pgoal, areaCosts, gpath_desired, "invMILP", verbose=False, acceptable_dist=0.0)
      results2+= benchmarkExplanationISP(G, pstart, pgoal, areaCosts, gpath_desired, "invMILPapprox", verbose=False, acceptable_dist=0.0)
      rospy.loginfo("Benchmark for polyCosts:")
      rospy.loginfo("\n"+str(tabulate(results1, headers="keys")))
      rospy.loginfo("Benchmark for polyLabels:")
      rospy.loginfo("\n"+str(tabulate(results2, headers="keys")))

