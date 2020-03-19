#!/usr/bin/env python

import networkx as nx
import numpy as np
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
  except rospy.ServiceException, e:
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

### main

if __name__ == "__main__":

  debug = False

  # ros init
  rospy.init_node('test_recast_explanations')
  pubGraph = rospy.Publisher('graph', MarkerArray, queue_size=10)
  pubPath = rospy.Publisher('graph_path', Marker, queue_size=10)
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
      km, datam = addPortal(rosgraph.portals[i/2], N)
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
    for i in range(0, len(rospath.points), 2) + [len(rospath.points)-1]:
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
    if False:
      rospy.loginfo('Solving k-shortest paths on our local graph...')
      time1 = time.clock()
      kpaths = list(islice(nx.shortest_simple_paths(G, pstart, pgoal, weight="weight"), 5))
      rospy.loginfo('... elapsed time: ' + str(time.clock()-time1))
    else:
      kpaths = []

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




