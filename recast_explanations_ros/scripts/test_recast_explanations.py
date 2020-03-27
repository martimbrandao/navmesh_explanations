#!/usr/bin/env python3

import networkx as nx
import numpy as np
import cvxpy as cp
import time
import pdb
import rospy
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from recast_ros.srv import RecastProjectSrv, RecastProjectSrvRequest
from recast_ros.msg import RecastGraph, RecastGraphNode
from itertools import islice

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

  # solve with cvxpy
  x = cp.Variable(len(areaCosts))
  cost = cp.sum_squares(x - np.array(areaCosts))
  prob = cp.Problem(cp.Minimize(cost), [G @ x <= h, x >= 0])
  prob.solve()
  newAreaCosts = x.value

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

  # solve with cvxpy
  x = cp.Variable(len(graph.nodes))
  cost = cp.sum_squares(x - np.array(nodeCosts))
  prob = cp.Problem(cp.Minimize(cost), [G @ x <= h, x >= 0])
  prob.solve()
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
    newGraph[edge[0]][edge[1]]["weight"] = cost * dist(newGraph.nodes[edge[0]]["point"], newGraph.nodes[edge[1]]["point"])
  return x.value, newGraph

def optPolyLabelsApproxFromCosts(graph, areaCosts, allowedAreaTypes, desiredPath, badPaths):
  # get optimal node costs
  nodeCosts,tmpGraph = optPolyCosts(graph, desiredPath, badPaths)

  # infer each node's area-type from cost
  newPolyLabels = [0]*len(graph.nodes)
  newGraph = graph.copy()
  for node in graph.nodes:
    area = graph.nodes[node]["area"]
    cost = nodeCosts[ graph.nodes[node]["id"] ]
    # if cost has changed then choose area type with closest cost
    if cost != graph.nodes[node]["cost"]:
      bestArea = -1
      bestDist = float('inf')
      for a in allowedAreaTypes:
        if a == area:
          continue
        d = abs(cost - areaCosts[a])
        if d < bestDist:
          bestDist = d
          bestArea = a
      newGraph.nodes[node]["area"] = bestArea
      newPolyLabels[ newGraph.nodes[node]["id"] ] = bestArea
    else:
      newPolyLabels[ newGraph.nodes[node]["id"] ] = cost
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

  # solve with cvxpy
  x = cp.Variable(len(nodeLabelsHotEnc), boolean=True)
  cost = cp.sum_squares(x - np.array(nodeLabelsHotEnc))
  prob = cp.Problem(cp.Minimize(cost), [G @ x <= h, A @ x == b])
  prob.solve(solver=cp.MOSEK, verbose=True, mosek_params={"MSK_DPAR_MIO_MAX_TIME":600})

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
    else:
      cost = newGraph.nodes[edge[1]]["cost"]
    newGraph[edge[0]][edge[1]]["weight"] = cost * dist(newGraph.nodes[edge[0]]["point"], newGraph.nodes[edge[1]]["point"])
  return newPolyLabels, newGraph

### main

if __name__ == "__main__":

  debug = False

  # ros init
  rospy.init_node('test_recast_explanations')
  pubGraph = rospy.Publisher('graph', MarkerArray, queue_size=10)
  pubPath = rospy.Publisher('graph_path', Marker, queue_size=10)
  pubPathDesired = rospy.Publisher('graph_path_desired', Marker, queue_size=10)
  pubKPaths = rospy.Publisher('graph_kpaths', Marker, queue_size=10)

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

    # k-shortest paths
    if True:
      rospy.loginfo('Solving k-shortest paths on our local graph...')
      time1 = time.clock()
      kpaths = list(islice(nx.shortest_simple_paths(G, pstart, pgoal, weight="weight"), 5))
      rospy.loginfo('... elapsed time: ' + str(time.clock()-time1))
    else:
      kpaths = []

    # desired path on graph
    rospy.loginfo('Solving shortest-hop ("desired") path on our local graph...')
    gpath_desired = nx.shortest_path(G, source=pstart, target=pgoal)

    # inverse shortest path
    if True:
      # optAreaCosts
      print("Computing optAreaCosts...")
      x1, G1 = optAreaCosts(G, areaCosts, gpath_desired, kpaths)
      print(x1)
      # optPolyCosts
      print("Computing optPolyCosts...")
      x2, G2 = optPolyCosts(G, gpath_desired, kpaths)
      print(x2)
      # optPolyLabelsApproxFromCosts
      print("Computing optPolyLabelsApproxFromCosts...")
      x3, G3 = optPolyLabelsApproxFromCosts(G, areaCosts, [1,2], gpath_desired, kpaths)
      print(x3)
      # optPolyLabels
      print("Computing optPolyLabels...")
      x4, G4 = optPolyLabels(G, areaCosts, gpath_desired, kpaths)
      print(x4)

    # visualize our graph
    rospy.loginfo('Visualizing our graph...')
    graph_height = 0.3

    graphmarker1 = Marker()
    graphmarker1.header.frame_id = 'map'
    graphmarker1.header.stamp = rospy.Time.now()
    graphmarker1.id = 0
    graphmarker1.action = Marker.ADD
    graphmarker1.scale.x = 0.05
    graphmarker1.type = Marker.LINE_LIST
    graphmarker1.color.r = 0
    graphmarker1.color.g = 0
    graphmarker1.color.b = 1
    graphmarker1.color.a = 1
    graphmarker1.pose.orientation.w = 1
    for edge in list(G.edges):
      if G[edge[0]][edge[1]]["area"] == 1:
        p1 = Point()
        p1.x = N[edge[0]]["point"].x
        p1.y = N[edge[0]]["point"].y
        p1.z = N[edge[0]]["point"].z + graph_height
        graphmarker1.points.append(p1)
        p2 = Point()
        p2.x = N[edge[1]]["point"].x
        p2.y = N[edge[1]]["point"].y
        p2.z = N[edge[1]]["point"].z + graph_height
        graphmarker1.points.append(p2)

    graphmarker2 = Marker()
    graphmarker2.header.frame_id = 'map'
    graphmarker2.header.stamp = rospy.Time.now()
    graphmarker2.id = 1
    graphmarker2.action = Marker.ADD
    graphmarker2.scale.x = 0.05
    graphmarker2.type = Marker.LINE_LIST
    graphmarker2.color.r = 1
    graphmarker2.color.g = 0
    graphmarker2.color.b = 1
    graphmarker2.color.a = 1
    graphmarker2.pose.orientation.w = 1
    for edge in list(G.edges):
      if G[edge[0]][edge[1]]["area"] == 2:
        p1 = Point()
        p1.x = N[edge[0]]["point"].x
        p1.y = N[edge[0]]["point"].y
        p1.z = N[edge[0]]["point"].z + graph_height
        graphmarker2.points.append(p1)
        p2 = Point()
        p2.x = N[edge[1]]["point"].x
        p2.y = N[edge[1]]["point"].y
        p2.z = N[edge[1]]["point"].z + graph_height
        graphmarker2.points.append(p2)

    nodemarker1 = Marker()
    nodemarker1.header.frame_id = 'map'
    nodemarker1.header.stamp = rospy.Time.now()
    nodemarker1.id = 2
    nodemarker1.action = Marker.ADD
    nodemarker1.scale.x = 0.2
    nodemarker1.scale.y = 0.2
    nodemarker1.scale.z = 0.2
    nodemarker1.type = Marker.SPHERE_LIST
    nodemarker1.color.r = 0
    nodemarker1.color.g = 0
    nodemarker1.color.b = 1
    nodemarker1.color.a = 1
    nodemarker1.pose.orientation.w = 1
    for node in list(G.nodes):
      if N[node]["area"] == 1:
        p = Point()
        p.x = N[node]["point"].x
        p.y = N[node]["point"].y
        p.z = N[node]["point"].z + graph_height
        nodemarker1.points.append(p)

    nodemarker2 = Marker()
    nodemarker2.header.frame_id = 'map'
    nodemarker2.header.stamp = rospy.Time.now()
    nodemarker2.id = 3
    nodemarker2.action = Marker.ADD
    nodemarker2.scale.x = 0.2
    nodemarker2.scale.y = 0.2
    nodemarker2.scale.z = 0.2
    nodemarker2.type = Marker.SPHERE_LIST
    nodemarker2.color.r = 1
    nodemarker2.color.g = 0
    nodemarker2.color.b = 1
    nodemarker2.color.a = 1
    nodemarker2.pose.orientation.w = 1
    for node in list(G.nodes):
      if N[node]["area"] == 2:
        p = Point()
        p.x = N[node]["point"].x
        p.y = N[node]["point"].y
        p.z = N[node]["point"].z + graph_height
        nodemarker2.points.append(p)

    nodemarker3 = Marker()
    nodemarker3.header.frame_id = 'map'
    nodemarker3.header.stamp = rospy.Time.now()
    nodemarker3.id = 4
    nodemarker3.action = Marker.ADD
    nodemarker3.scale.x = 0.1
    nodemarker3.scale.y = 0.1
    nodemarker3.scale.z = 0.1
    nodemarker3.type = Marker.SPHERE_LIST
    nodemarker3.color.r = 0
    nodemarker3.color.g = 1
    nodemarker3.color.b = 0
    nodemarker3.color.a = 1
    nodemarker3.pose.orientation.w = 1
    for node in list(G.nodes):
      if N[node]["area"] == -1:
        p = Point()
        p.x = N[node]["point"].x
        p.y = N[node]["point"].y
        p.z = N[node]["point"].z + graph_height
        nodemarker3.points.append(p)

    # publish graph and nodes
    graphmarkers = MarkerArray()
    graphmarkers.markers.append(graphmarker1)
    graphmarkers.markers.append(graphmarker2)
    graphmarkers.markers.append(nodemarker1)
    graphmarkers.markers.append(nodemarker2)
    graphmarkers.markers.append(nodemarker3)
    if pubGraph.get_num_connections() > 0:
      pubGraph.publish(graphmarkers)

    # visualize graph path
    rospy.loginfo('Visualizing our shortest path...')
    marker = Marker()
    marker.header.frame_id = 'map'
    marker.header.stamp = rospy.Time.now()
    marker.id = 0
    marker.action = Marker.ADD
    marker.scale.x = 0.1
    marker.type = Marker.LINE_LIST
    marker.color.r = 1
    marker.color.g = 0
    marker.color.b = 0
    marker.color.a = 1
    marker.pose.orientation.w = 1
    for i in range(len(gpath)-1):
      p1 = Point()
      p1.x = N[gpath[i  ]]["point"].x
      p1.y = N[gpath[i  ]]["point"].y
      p1.z = N[gpath[i  ]]["point"].z + 1
      marker.points.append(p1)
      p2 = Point()
      p2.x = N[gpath[i+1]]["point"].x
      p2.y = N[gpath[i+1]]["point"].y
      p2.z = N[gpath[i+1]]["point"].z + 1
      marker.points.append(p2)
    if pubPath.get_num_connections() > 0:
      pubPath.publish(marker)

    # visualize graph K paths
    if len(kpaths) > 0:
      rospy.loginfo('Visualizing our k-shortest paths...')
      marker = Marker()
      marker.header.frame_id = 'map'
      marker.header.stamp = rospy.Time.now()
      marker.id = 0
      marker.action = Marker.ADD
      marker.scale.x = 0.1
      marker.type = Marker.LINE_LIST
      marker.color.r = 1
      marker.color.g = 1
      marker.color.b = 1
      marker.color.a = 0.8
      marker.pose.orientation.w = 1
      for path in kpaths:
        for i in range(len(path)-1):
          p1 = Point()
          p1.x = N[path[i  ]]["point"].x
          p1.y = N[path[i  ]]["point"].y
          p1.z = N[path[i  ]]["point"].z + 0.9
          marker.points.append(p1)
          p2 = Point()
          p2.x = N[path[i+1]]["point"].x
          p2.y = N[path[i+1]]["point"].y
          p2.z = N[path[i+1]]["point"].z + 0.9
          marker.points.append(p2)
      if pubKPaths.get_num_connections() > 0:
        pubKPaths.publish(marker)

    # visualize desired graph path
    rospy.loginfo('Visualizing our desired path...')
    marker = Marker()
    marker.header.frame_id = 'map'
    marker.header.stamp = rospy.Time.now()
    marker.id = 0
    marker.action = Marker.ADD
    marker.scale.x = 0.1
    marker.type = Marker.LINE_LIST
    marker.color.r = 0
    marker.color.g = 1
    marker.color.b = 0
    marker.color.a = 1
    marker.pose.orientation.w = 1
    for i in range(len(gpath_desired)-1):
      p1 = Point()
      p1.x = N[gpath_desired[i  ]]["point"].x
      p1.y = N[gpath_desired[i  ]]["point"].y
      p1.z = N[gpath_desired[i  ]]["point"].z + 1
      marker.points.append(p1)
      p2 = Point()
      p2.x = N[gpath_desired[i+1]]["point"].x
      p2.y = N[gpath_desired[i+1]]["point"].y
      p2.z = N[gpath_desired[i+1]]["point"].z + 1
      marker.points.append(p2)
    if pubPathDesired.get_num_connections() > 0:
      pubPathDesired.publish(marker)



