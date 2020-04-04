#!/usr/bin/env python3

import networkx as nx
import numpy as np
import cvxpy as cp
import similaritymeasures
import matplotlib.pyplot as plt
import time
import random
import pdb
import rospy
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from recast_ros.srv import RecastProjectSrv, RecastProjectSrvRequest
from recast_ros.msg import RecastGraph, RecastGraphNode
from itertools import islice
from tabulate import tabulate


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
  graphmarkers.markers.append(graphmarker1)
  graphmarkers.markers.append(graphmarker2)
  graphmarkers.markers.append(nodemarker1)
  graphmarkers.markers.append(nodemarker2)
  #graphmarkers.markers.append(nodemarker3)
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


### optimization

def optAreaCosts(graph, areaCosts, desiredPath, badPaths):
  # variable:     ac = areaCosts                                                  # vector of all area-label costs (float)
  # cost:         (ac-areaCosts)^2
  # constraints:  sum_(j in B) dist_j * ac_j - sum_(j in S_i) dist_j * ac_j <= 0  # for all bad paths S_i
  #               ac_i >= 0                                                       # for all i

  # path cost constraints
  G = []
  h = []
  for path in badPaths:
    line = [0]*len(areaCosts)
    # sum_(j in B) dist_j * ac_j
    for i in range(len(desiredPath)-1):
      d = dist(graph.nodes[desiredPath[i]]["point"], graph.nodes[desiredPath[i+1]]["point"])
      if graph.nodes[desiredPath[i]]["portal"] == False:
        line[ graph.nodes[desiredPath[i]]["area"] ] += d
      elif graph.nodes[desiredPath[i+1]]["portal"] == False:
        line[ graph.nodes[desiredPath[i+1]]["area"] ] += d
    # - sum_(j in S_i) dist_j * ac_j
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
  nodeCosts = np.array([0] * len(graph.nodes))
  for node in graph.nodes:
    nodeCosts[ graph.nodes[node]["id"] ] = graph.nodes[node]["cost"]

  # path cost constraints
  G = []
  h = []
  for path in badPaths:
    line = [0]*len(nodeCosts)
    # sum_(j in B) dist_j * nc_j
    for i in range(len(desiredPath)-1):
      d = dist(graph.nodes[desiredPath[i]]["point"], graph.nodes[desiredPath[i+1]]["point"])
      if graph.nodes[desiredPath[i]]["portal"] == False:
        line[ graph.nodes[desiredPath[i]]["id"] ] += d
      elif graph.nodes[desiredPath[i+1]]["portal"] == False:
        line[ graph.nodes[desiredPath[i+1]]["id"] ] += d
    # - sum_(j in S_i) dist_j * nc_j
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
  nodeLabelsHotEnc = np.array([0] * len(graph.nodes)*2)
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
    line = [0]*len(nodeLabelsHotEnc)
    # sum_(j in B) dist_i * ac_0 * l_0 + dist_i * ac_1 * l_1
    for i in range(len(desiredPath)-1):
      d = dist(graph.nodes[desiredPath[i]]["point"], graph.nodes[desiredPath[i+1]]["point"])
      if graph.nodes[desiredPath[i]]["portal"] == False:
        line[ graph.nodes[desiredPath[i]]["id"]*2   ] += d * areaCosts[1]
        line[ graph.nodes[desiredPath[i]]["id"]*2+1 ] += d * areaCosts[2]
      elif graph.nodes[desiredPath[i+1]]["portal"] == False:
        line[ graph.nodes[desiredPath[i+1]]["id"]*2   ] += d * areaCosts[1]
        line[ graph.nodes[desiredPath[i+1]]["id"]*2+1 ] += d * areaCosts[2]
    # - sum_(j in S_i) dist_i * ac_0 * l_0 + dist_i * ac_1 * l_1
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

  # path cost constraints
  G = []
  h = []
  for path in badPaths:
    line = [0]*len(nodeLabelsHotEnc)
    # sum_(j in B) dist_i * ac_0 * l_0 + dist_i * ac_1 * l_1
    for i in range(len(desiredPath)-1):
      d = dist(graph.nodes[desiredPath[i]]["point"], graph.nodes[desiredPath[i+1]]["point"])
      if graph.nodes[desiredPath[i]]["portal"] == False:
        line[ auxgraph.nodes[desiredPath[i]]["varidx"]   ] += d * areaCosts[1]
        line[ auxgraph.nodes[desiredPath[i]]["varidx"]+1 ] += d * areaCosts[2]
      elif graph.nodes[desiredPath[i+1]]["portal"] == False:
        line[ auxgraph.nodes[desiredPath[i+1]]["varidx"]   ] += d * areaCosts[1]
        line[ auxgraph.nodes[desiredPath[i+1]]["varidx"]+1 ] += d * areaCosts[2]
    # - sum_(j in S_i) dist_i * ac_0 * l_0 + dist_i * ac_1 * l_1
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
  cost = cp.norm1(x - np.array(nodeLabelsHotEnc)) + 1 * cp.maximum(cp.max(G @ x - h), 0)
  prob = cp.Problem(cp.Minimize(cost), [A @ x == b])
  prob.solve(solver=cp.MOSEK, mosek_params={"MSK_DPAR_MIO_MAX_TIME":90})

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


def computeExplanationISP(graph, start, goal, area_costs, desired_path, problem_type, diversity_method, num_alternatives, max_iter, verbose=True, acceptable_dist=0.0):

  if verbose:
    rospy.loginfo("Computing explanation type %s, using %d %s-diverse paths and %d iterations..." % (problem_type, num_alternatives, diversity_method, max_iter))

  # check shortest path
  path = nx.shortest_path(graph, source=start, target=goal, weight="weight")
  if path == desired_path:
    rospy.loginfo("Shortest path is already equal to desired. Nothing to explain.")
    return True, [], graph.copy()

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

    all_alternative_paths += alternative_paths

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
    time3 = time.clock()

    # compute new shortest path
    path = nx.shortest_path(newgraph, source=start, target=goal, weight="weight")

    # save
    history_explanations.append( newgraph.copy() )
    history_explanation_distances.append( pathDistance(desired_path, path) )
    if verbose:
      rospy.loginfo("  iteration %d took %f + %f secs. Similarity to desired path = %f" % (it, time2-time1, time3-time2, pathDistance(desired_path, path)))

    # check that desired path is the shortest in new graph
    if pathDistance(desired_path, path) < acceptable_dist:
      desired_is_shortest = True
      break

  # pick best explanation (since performance usually fluctuates as we add more alternative paths)
  best_explanation = np.argmin(history_explanation_distances)
  newgraph = history_explanations[best_explanation]
  if verbose:
    rospy.loginfo("...finished %s in %d iterations. Best explanation leads to a shortest-desired path distance = %f" % (problem_type, it+1, history_explanation_distances[best_explanation]))

  return desired_is_shortest, x, newgraph


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

  results = []
  for diversity_method in ["ksp", "kdp-brandao"]:
    for num_alternatives in [1, 2, 5, 10, 20]:
      for max_iter in [10]:

        # compute explanation
        time1 = time.clock()
        ok, x, newgraph = computeExplanationISP(graph, start, goal, area_costs, desired_path, problem_type, diversity_method, num_alternatives, max_iter, verbose, acceptable_dist)
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
          for node in graph:
            x1.append(graph.nodes[node]["cost"])
            x2.append(newgraph.nodes[node]["cost"])
          x1 = np.array(x1)
          x2 = np.array(x2)
        else: # polyLabels and approximations
          x1 = []
          x2 = []
          for node in graph:
            x1.append(graph.nodes[node]["area"])
            x2.append(newgraph.nodes[node]["area"])
          x1 = np.array(x1)
          x2 = np.array(x2)
        l1norm = np.linalg.norm(x2 - x1, 1)
        changed = np.sum( np.abs(x2 - x1) > 0.01 )

        # log results
        result = {}
        result["diversity_method"] = diversity_method
        result["num_alternatives"] = num_alternatives
        result["max_iter"] = max_iter
        result["graph"] = newgraph
        result["l1_norm"] = l1norm
        result["changed_vars"] = changed
        result["dist"] = pathDistance(desired_path, path)
        result["time"] = time2 - time1
        results.append(result)

  rospy.loginfo("Results:\n" + str(tabulate(results, headers="keys")))


### main

if __name__ == "__main__":

  debug = False

  # ros init
  rospy.init_node('test_recast_explanations')

  pubGraph = rospy.Publisher('graph', MarkerArray, queue_size=10)
  pubPath = rospy.Publisher('graph_path', Marker, queue_size=10)
  pubPathDesired = rospy.Publisher('graph_path_desired', Marker, queue_size=10)

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

  rospy.loginfo('Waiting for recast_ros...')
  rospy.wait_for_service('/recast_node/plan_path')
  rospy.wait_for_service('/recast_node/project_point')

  # loop
  rate = rospy.Rate(1.0) # hz
  while not rospy.is_shutdown():
    rate.sleep()

    rospy.loginfo('=======================================================')

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
      G.add_edge(k1, km, weight=cost1, area=data1["area"])
      G.add_edge(km, k2, weight=cost2, area=data2["area"])
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

    # debug
    if debug:
      rospy.loginfo('start    = \n' + str(rospath_centers[0]))
      rospy.loginfo('goal     = \n' + str(rospath_centers[-1]))
      rospy.loginfo('my start = \n' + str(pstart))
      rospy.loginfo('my goal  = \n' + str(pgoal))

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

    # desired path on graph
    rospy.loginfo('Solving shortest-hop ("desired") path on our local graph...')
    gpath_desired = nx.shortest_path(G, source=pstart, target=pgoal)

    # visualize graph and paths
    if pubGraph.get_num_connections() > 0:
      rospy.loginfo('Visualizing our graph...')
      #pubGraph.publish( graphToMarkerArrayByArea(G, 0.2, list(range(-1,len(areaCosts)))) )
      pubGraph.publish( graphToMarkerArray(G, 0.2) )

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
      ok1, x1, G1 = computeExplanationISP(G, pstart, pgoal, areaCosts, gpath_desired, "areaCosts", "kdp-brandao", 2, 10, verbose=False, acceptable_dist=3.0)
      xpath1 = nx.shortest_path(G1, source=pstart, target=pgoal, weight="weight")
      # visualize
      pubAreaCostsPath.publish( pathToMarker(G1, xpath1, 0, [1,0,0,1], 0.9) )
      pubAreaCostsGraph.publish( graphToMarkerArrayByCost(G1, 0.2) )

    if pubPolyCostsPath.get_num_connections() > 0 or pubPolyCostsGraph.get_num_connections() > 0:
      rospy.loginfo("Computing explanation based on polyCosts...")
      ok2, x2, G2 = computeExplanationISP(G, pstart, pgoal, areaCosts, gpath_desired, "polyCosts", "kdp-brandao", 2, 10, verbose=False, acceptable_dist=3.0)
      xpath2 = nx.shortest_path(G2, source=pstart, target=pgoal, weight="weight")
      # visualize
      pubPolyCostsPath.publish( pathToMarker(G2, xpath2, 0, [1,0,0,1], 0.9) )
      pubPolyCostsGraph.publish( graphToMarkerArrayByCost(G2, 0.2) )

    if pubPolyLabelsPath.get_num_connections() > 0 or pubPolyLabelsGraph.get_num_connections() > 0:
      rospy.loginfo("Computing explanation based on polyLabelsInPath...")
      ok3, x3, G3 = computeExplanationISP(G, pstart, pgoal, areaCosts, gpath_desired, "polyLabelsInPath", "kdp-brandao", 2, 10, verbose=False, acceptable_dist=3.0)
      xpath3 = nx.shortest_path(G3, source=pstart, target=pgoal, weight="weight")
      # visualize
      pubPolyLabelsPath.publish( pathToMarker(G3, xpath3, 0, [1,0,0,1], 0.9) )
      pubPolyLabelsGraph.publish( graphToMarkerArray(G3, 0.2) )
      #pubPolyLabelsGraph.publish( graphToMarkerArrayByArea(G3, 0.2, list(range(-1,len(areaCosts)))) )

    # Running bencharks
    if False:
      rospy.loginfo("Running benchmarks for each problem type...")
      benchmarkExplanationISP(G, pstart, pgoal, areaCosts, gpath_desired, "areaCosts", verbose=False, acceptable_dist=5.0)
      benchmarkExplanationISP(G, pstart, pgoal, areaCosts, gpath_desired, "polyCosts", verbose=False, acceptable_dist=5.0)
      benchmarkExplanationISP(G, pstart, pgoal, areaCosts, gpath_desired, "polyLabelsInPath", verbose=False, acceptable_dist=5.0)
      benchmarkExplanationISP(G, pstart, pgoal, areaCosts, gpath_desired, "polyLabels", verbose=False, acceptable_dist=5.0)

      rospy.loginfo("Running benchmark [polyLabelsInPath VS polyLabels] for maximum number of iterations...")
      benchmarkExplanationISP(G, pstart, pgoal, areaCosts, gpath_desired, "polyLabelsInPath", verbose=False, acceptable_dist=0.0)
      benchmarkExplanationISP(G, pstart, pgoal, areaCosts, gpath_desired, "polyLabels", verbose=False, acceptable_dist=0.0)

