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
  markers = MarkerArray()
  for i in range(len(paths)):
    m = pathToMarker(graph, paths[i], i, [1,1,1,0.8], 0.9)
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
  for node in list(G.nodes):
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
  graphmarkers.markers.append(nodemarker3)
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
  prob.solve(solver=cp.MOSEK, verbose=True, mosek_params={"MSK_DPAR_MIO_MAX_TIME":300})

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
  pubKPaths = rospy.Publisher('graph_kpaths', MarkerArray, queue_size=10)
  pubXPath1 = rospy.Publisher('contrastive_path1', Marker, queue_size=10)
  pubXPath2 = rospy.Publisher('contrastive_path2', Marker, queue_size=10)
  pubXPath3 = rospy.Publisher('contrastive_path3', Marker, queue_size=10)
  pubXPath4 = rospy.Publisher('contrastive_path4', Marker, queue_size=10)

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
      rospy.loginfo("Computing optAreaCosts...")
      x1, G1 = optAreaCosts(G, areaCosts, gpath_desired, kpaths)
      xpath1 = nx.shortest_path(G1, source=pstart, target=pgoal, weight="weight")

      # optPolyCosts
      rospy.loginfo("Computing optPolyCosts...")
      x2, G2 = optPolyCosts(G, gpath_desired, kpaths)
      xpath2 = nx.shortest_path(G2, source=pstart, target=pgoal, weight="weight")

      # optPolyLabelsApproxFromCosts
      rospy.loginfo("Computing optPolyLabelsApproxFromCosts...")
      x3, G3 = optPolyLabelsApproxFromCosts(G, areaCosts, [1,2], gpath_desired, kpaths)
      xpath3 = nx.shortest_path(G3, source=pstart, target=pgoal, weight="weight")

      # optPolyLabels
      #rospy.loginfo("Computing optPolyLabels...")
      #x4, G4 = optPolyLabels(G, areaCosts, gpath_desired, kpaths)
      #xpath4 = nx.shortest_path(G4, source=pstart, target=pgoal, weight="weight")

    # visualize graph and paths
    if pubGraph.get_num_connections() > 0:
      rospy.loginfo('Visualizing our graph...')
      pubGraph.publish( graphToMarkerArray(graph, 0.3) )

    if pubPath.get_num_connections() > 0:
      rospy.loginfo('Visualizing our shortest path...')
      pubPath.publish( pathToMarker(G, gpath, 0, [1,0.5,0,1], 1) )

    if len(kpaths) > 0 and pubKPaths.get_num_connections() > 0:
      rospy.loginfo('Visualizing our k-shortest paths...')
      pubKPaths.publish( pathsToMarkerArray(G, kpaths, 0.9) )

    if pubPathDesired.get_num_connections() > 0:
      rospy.loginfo('Visualizing our desired path...')
      pubPathDesired.publish( pathToMarker(G, path_desired, 0, [0,1,0,1], 1) )

    # visualize contrastive paths
    if pubXPath1.get_num_connections() > 0:
      rospy.loginfo('Visualizing contrastive path1...')
      pubXPath1.publish( pathToMarker(G1, xpath1, 0, [1,0,0,1], 0.4) )

    if pubXPath2.get_num_connections() > 0:
      rospy.loginfo('Visualizing contrastive path2...')
      pubXPath2.publish( pathToMarker(G2, xpath2, 0, [0,1,0,1], 0.4) )

    if pubXPath3.get_num_connections() > 0:
      rospy.loginfo('Visualizing contrastive path3...')
      pubXPath3.publish( pathToMarker(G3, xpath3, 0, [0,0,1,1], 0.4) )

    #if pubXPath4.get_num_connections() > 0:
    #  rospy.loginfo('Visualizing contrastive path4...')
    #  pubXPath4.publish( pathToMarker(G4, xpath4, 0, [1,1,1,1], 0.4) )

