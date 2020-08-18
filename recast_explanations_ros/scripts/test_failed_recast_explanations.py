#!/usr/bin/env python3

# TODO:
# - recast_ros create my own launch/cfg that I can edit
# - recast_ros publish both graph and graph_filtered, change rviz config to show filtered
# - interface can publish both a marker and marker_filtered
# - publish explanation visualization here (marker with lines)
# - test to find issues. predicted: shortest-bridge might be through a wall (hard), should also show shortest-freespace-bridge (need raycasting or similar here)

import networkx as nx
import numpy as np
import cvxpy as cp
import scipy
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
from std_msgs.msg import ColorRGBA
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
  data["idx_tri"] = rosnode.idx_triangles.data
  data["num_tri"] = rosnode.num_triangles.data
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
  if color != None and len(color) == 4:
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


def graphToNavmesh(graph, navmesh, area2color):
  newnavmesh = newMarker(navmesh.id, navmesh.type, 1, None)
  for node in graph.nodes:
    if not graph.nodes[node]["portal"]:
      idx = graph.nodes[node]["idx_tri"]
      num = graph.nodes[node]["num_tri"]
      color = area2color[graph.nodes[node]["area"]]
      for i in range(idx, idx+num*3):
        roscolor = ColorRGBA()
        roscolor.r = color[0]
        roscolor.g = color[1]
        roscolor.b = color[2]
        roscolor.a = color[3]
        newnavmesh.points.append(navmesh.points[i])
        newnavmesh.colors.append(roscolor)
  return newnavmesh


def graphToMarkerArray(graph, height):
  # darkness
  dark = 0.85
  # edges
  graphmarker1 = newMarker(0, Marker.LINE_LIST, 0.05, [0.6509804129600525*dark, 0.7411764860153198*dark, 0.843137264251709*dark, 1.0])
  graphmarker2 = newMarker(1, Marker.LINE_LIST, 0.05, [0.7568627595901489*dark, 0.0,                     0.125490203499794*dark, 1.0])
  for edge in list(graph.edges):
    if graph[edge[0]][edge[1]]["area"] == 1:
      graphmarker1.points.append(newPoint(graph.nodes[edge[0]]["point"], height))
      graphmarker1.points.append(newPoint(graph.nodes[edge[1]]["point"], height))
    elif graph[edge[0]][edge[1]]["area"] == 2:
      graphmarker2.points.append(newPoint(graph.nodes[edge[0]]["point"], height))
      graphmarker2.points.append(newPoint(graph.nodes[edge[1]]["point"], height))
  # nodes
  nodemarker1 = newMarker(2, Marker.SPHERE_LIST, 0.2, [0.6509804129600525*dark, 0.7411764860153198*dark, 0.843137264251709*dark, 1.0])
  nodemarker2 = newMarker(3, Marker.SPHERE_LIST, 0.2, [0.7568627595901489*dark, 0.0,                     0.125490203499794*dark, 1.0])
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


### failure explanations

def subgraph_distance(graph, group1, group2, kdtree2):
  # for each point in group1
  mindist = float('inf')
  for node1 in group1:
    # find closest point in group2
    d, idx = kdtree2.query(node1)
    if d < mindist:
      mindist = d
      closest_node1 = node1
      closest_node2 = group2[idx]
  return mindist, closest_node1, closest_node2


def explain(graph, start, goal):
  # TODO: A*search version of this whole thing (doesnt compute distance matrix, does it on the go)
  # get connected components or "islands"
  islands = []
  kdtrees = []
  for c in sorted(nx.connected_components(graph), key=len, reverse=True):
    islands.append(list(c))
    kdtrees.append(scipy.spatial.KDTree(list(c)))
  # create bridge hypotheses
  bridge_graph = nx.Graph()
  # distance matrix
  distances = dict()
  D = np.zeros([len(islands), len(islands)])
  for i in range(len(islands)):
    for j in range(len(islands)):
      if i <= j:
        continue
      mindist, closest_node1, closest_node2 = subgraph_distance(graph, islands[i], islands[j], kdtrees[j])
      distances[i,j] = (mindist, closest_node1, closest_node2)
      distances[j,i] = (mindist, closest_node1, closest_node2)
      D[i,j] = mindist
      bridge_graph.add_edge(i, j, weight=mindist)
  # start and goal islands
  start_island = [start in islands[i] for i in range(len(islands))].index(True)
  goal_island = [goal in islands[i] for i in range(len(islands))].index(True)
  # find minimum distance that connects start/goal
  fewest_bridges_path = nx.shortest_path(bridge_graph, source=start_island, target=goal_island)
  shortest_bridges_path = nx.shortest_path(bridge_graph, source=start_island, target=goal_island, weight="weight")
  rospy.loginfo('   fewest bridges to connect start/goal   : %s  (%f meters)' % (fewest_bridges_path, getCost(bridge_graph, fewest_bridges_path)))
  rospy.loginfo('   shortest bridges to connects start/goal: %s  (%f meters)' % (shortest_bridges_path, getCost(bridge_graph, shortest_bridges_path)))
  return


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
  rospy.init_node('failed_recast_explanations')

  pubGraph = rospy.Publisher('graph', MarkerArray, queue_size=10)
  pubGraphCosts = rospy.Publisher('graph_costs', MarkerArray, queue_size=10)
  pubPath = rospy.Publisher('graph_path', Marker, queue_size=10)
  pubPathDesired = rospy.Publisher('graph_path_desired', Marker, queue_size=10)

  rospy.loginfo('Waiting for recast_ros...')
  rospy.wait_for_service('/recast_node/plan_path')
  rospy.wait_for_service('/recast_node/project_point')

  rosgraph = None
  areaCosts = None
  pstart = None
  pgoal = None
  old_rosgraph = None
  old_areaCosts = None
  old_pstart = None
  old_pgoal = None

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

    rospy.loginfo("Graph |V| = %d, |E| = %d " % (len(G.nodes), len(G.edges)))

    # get navmesh
    rospy.loginfo('Getting navmesh...')
    navmesh = rospy.wait_for_message('/recast_node/navigation_mesh', Marker)
    # find all colors
    navmesh_colors = list(set([(c.r,c.g,c.b,c.a) for c in navmesh.colors]))
    navmesh_areas = [0] * len(navmesh_colors)
    # infer area type of each color
    for c in range(len(navmesh_colors)):
      for i in range(len(navmesh.colors)):
        color = navmesh.colors[i]
        if (color.r,color.g,color.b,color.a) == navmesh_colors[c]:
          pt0 = navmesh.points[i]
          pt1 = navmesh.points[i+1]
          pt2 = navmesh.points[i+2]
          pt = Point()
          pt.x = (pt0.x + pt1.x + pt2.x)/3
          pt.y = (pt0.y + pt1.y + pt2.y)/3
          pt.z = (pt0.z + pt1.z + pt2.z)/3
          proj = getProj(pt)
          navmesh_areas[c] = proj.area_type.data
          break
    area2color = {}
    for i in range(len(navmesh_areas)):
      area2color[navmesh_areas[i]] = navmesh_colors[i]

    # get start and goal
    rospy.loginfo('Getting start and goal...')
    rstart = rospy.wait_for_message('/recast_node/recast_path_start', Marker)
    rgoal = rospy.wait_for_message('/recast_node/recast_path_goal', Marker)
    proj_start = getProj(rstart.pose.position)
    proj_goal = getProj(rgoal.pose.position)
    pstart = getKey(proj_start.projected_polygon_center)
    pgoal = getKey(proj_goal.projected_polygon_center)

    # path on graph
    rospy.loginfo('Solving shortest path on our local graph...')
    try:
      gpath = nx.shortest_path(G, source=pstart, target=pgoal, weight="weight")
      found_path = True
      rospy.loginfo('...path found')
    except nx.NetworkXNoPath:
      found_path = False
      rospy.loginfo('...path not found')
    if found_path:
      continue

    # explanation
    rospy.loginfo('Generating explanation...')
    explain(G, pstart, pgoal)

