//
// Copyright (c) 2019 Martim Brandão martim@robots.ox.ac.uk, Omer Burak Aladag aladagomer@sabanciuniv.edu, Ioannis Havoutis ioannis@robots.ox.ac.uk
// As a part of Dynamic Robot Systems Group, Oxford Robotics Institute, University of Oxford
//
// This software is provided 'as-is', without any express or implied
// warranty. In no event will the authors be held liable for any damages
// arising from the use of this software.
// Permission is granted to anyone to use this software for any purpose,
// including commercial applications, and to alter it and redistribute it
// freely, subject to the following restrictions:
// 1. The origin of this software must not be misrepresented; you must not
//   claim that you wrote the original software. If you use this software
//   in a product, an acknowledgment in the product documentation would be
//   appreciated but is not required.
// 2. Altered source versions must be plainly marked as such, and must not be
//   misrepresented as being the original software.
// 3. This notice may not be removed or altered from any source distribution.

#include "recast_ros/RecastPathSrv.h"
#include "recast_ros/RecastProjectSrv.h"
#include <std_srvs/Trigger.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/InteractiveMarkerControl.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <ros/package.h>
#include <algorithm>
#include <iterator>
#include <vector>
#include <fstream>

struct RecastNode
{
  RecastNode(ros::NodeHandle &nodeHandle)
      : nodeHandle_(nodeHandle), noAreaTypes_(20), colourList_(noAreaTypes_), loopRate_(1.0)
  {
    // contrastive position publisher
    ContrastivePub_ = nodeHandle_.advertise<geometry_msgs::Point>("contrastive_position", 1);

    // ROS::Rviz marker subscriber
    NavMeshFilteredSub_ = nodeHandle_.subscribe<visualization_msgs::Marker>("/recast_node/filtered_navigation_mesh", 1, &RecastNode::navmeshCallback, this);

    // service clients
    servicePlan_ = nodeHandle_.serviceClient<recast_ros::RecastPathSrv>("/recast_node/plan_path");
    serviceProject_ = nodeHandle_.serviceClient<recast_ros::RecastProjectSrv>("/recast_node/project_point");

    // ROS::Rviz interactive marker
    interactiveMarkerServer_.reset(new interactive_markers::InteractiveMarkerServer("navmesh_explanations", "", false));

    // create colour list for area types
    colourList_ = {
        UIntToColor(0xFFFF6800), //Vivid Orange
        UIntToColor(0xFFA6BDD7), //Very Light Blue
        UIntToColor(0xFFC10020), //Vivid Red
        UIntToColor(0xFFCEA262), //Grayish Yellow
        UIntToColor(0xFF817066), //Medium Gray
        UIntToColor(0xFFFFB300), //Vivid Yellow
        UIntToColor(0xFF803E75), //Strong Purple

        //The following will not be good for people with defective color vision
        UIntToColor(0xFF93AA00), //Vivid Yellowish Green
        UIntToColor(0xFF007D34), //Vivid Green
        UIntToColor(0xFFF6768E), //Strong Purplish Pink
        UIntToColor(0xFF00538A), //Strong Blue
        UIntToColor(0xFFFF7A5C), //Strong Yellowish Pink
        UIntToColor(0xFF53377A), //Strong Violet
        UIntToColor(0xFFFF8E00), //Vivid Orange Yellow
        UIntToColor(0xFFB32851), //Strong Purplish Red
        UIntToColor(0xFFF4C800), //Vivid Greenish Yellow
        UIntToColor(0xFF7F180D), //Strong Reddish Brown
        UIntToColor(0xFF593315), //Deep Yellowish Brown
        UIntToColor(0xFFF13A13), //Vivid Reddish Orange
        UIntToColor(0xFF232C16), //Dark Olive Green
    };
  }

  std_msgs::ColorRGBA UIntToColor(uint32_t color)
  {
    std_msgs::ColorRGBA c;
    c.a = ((double)((uint8_t)(color >> 24))) / 255.0;
    c.r = ((double)((uint8_t)(color >> 16))) / 255.0;
    c.g = ((double)((uint8_t)(color >> 8))) / 255.0;
    c.b = ((double)((uint8_t)(color >> 0))) / 255.0;
    return c;
  }

  void navmeshCallback(const visualization_msgs::Marker::ConstPtr& msg) {
    if (msg->id != navMeshFiltered_.id) {
      ROS_INFO("New NavMesh (id: %d)", msg->id);
      navMeshFiltered_ = *msg;
      updateMesh_ = true;
    }
  }

  void findPathPlanner()
  {
    if (!startSet_ || !goalSet_)
      return;
    ROS_INFO("Calling path planning service...");
    geometry_msgs::Point firstArr, secondArr;
    firstArr.x = startX_;
    firstArr.y = startY_;
    firstArr.z = startZ_;
    secondArr.x = goalX_;
    secondArr.y = goalY_;
    secondArr.z = goalZ_;
    recast_ros::RecastPathSrv srv;
    srv.request.startXYZ = firstArr;
    srv.request.goalXYZ = secondArr;
    if (!servicePlan_.call(srv))
      ROS_ERROR("Failed to call planning service...");
  }

  // Interactive markers for triangles
  void pathPlannerMenu()
  {
    // make navmesh transparent
    for (unsigned int i = 0; i < navMeshFiltered_.colors.size(); i++)
      navMeshFiltered_.colors[i].a = 0.5;

    // prepare interactive marker
    visualization_msgs::InteractiveMarker intMarker;
    visualization_msgs::InteractiveMarkerControl intControl;

    intControl.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
    intControl.always_visible = true;
    intMarker.scale = 0.2;
    intMarker.header.frame_id = "map";

    intMarker.name = navMeshFiltered_.ns + "Interactive";
    intControl.markers.push_back(navMeshFiltered_);
    intMarker.controls.push_back(intControl);
    interactiveMarkerServer_->insert(intMarker);

    menuHandler_.apply(*interactiveMarkerServer_, navMeshFiltered_.ns + "Interactive");
    interactiveMarkerServer_->applyChanges();
  }

  void setStart(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
  {
    ROS_INFO("Updating start...");
    startX_ = feedback->mouse_point.x;
    startY_ = feedback->mouse_point.y;
    startZ_ = feedback->mouse_point.z;
    startSet_ = true;
    updateStart_ = true;
  }

  void setGoal(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
  {
    ROS_INFO("Updating goal...");
    goalX_ = feedback->mouse_point.x;
    goalY_ = feedback->mouse_point.y;
    goalZ_ = feedback->mouse_point.z;
    goalSet_ = true;
    updateGoal_ = true;
  }

  void setWhyNot(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
  {
    ROS_INFO("Updating contrastive waypoint...");
    contrastiveX_ = feedback->mouse_point.x;
    contrastiveY_ = feedback->mouse_point.y;
    contrastiveZ_ = feedback->mouse_point.z;
    contrastiveSet_ = true;
    geometry_msgs::Point point;
    recast_ros::RecastProjectSrv srv;
    srv.request.point.x = contrastiveX_;
    srv.request.point.y = contrastiveY_;
    srv.request.point.z = contrastiveZ_;
    if (serviceProject_.call(srv)) {
      contrastiveX_ = srv.response.projected_point.x;
      contrastiveY_ = srv.response.projected_point.y;
      contrastiveZ_ = srv.response.projected_point.z;
    } else {
      ROS_ERROR("Could not call projection service");
    }
  }

  // Drop down menu for interactive markers
  // Triangles are handled by menuHandler_
  void initMenu()
  {
    setStartPos_ = menuHandler_.insert("Set Start Position", boost::bind(&RecastNode::setStart, this, _1));
    setGoalPos_ = menuHandler_.insert("Set Goal Position", boost::bind(&RecastNode::setGoal, this, _1));
    setContrastivePos_ = menuHandler_.insert("Set Contrastive Waypoint: Why not through here?", boost::bind(&RecastNode::setWhyNot, this, _1));
  }

  void run()
  {
    // Visualization
    initMenu();

    // Main loop
    while (ros::ok())
    {
      // update mesh
      if (updateMesh_) {
        ROS_INFO("Resetting interactive markers...");
        interactiveMarkerServer_.reset(new interactive_markers::InteractiveMarkerServer("navmesh_explanations", "", false));
        pathPlannerMenu();
        updateMesh_ = false;
      }
      // path planning request
      if (updateStart_ || updateGoal_) {
        findPathPlanner();
        updateStart_ = false;
        updateGoal_ = false;
      }
      // send contrastive position
      if (contrastiveSet_) {
        ROS_INFO("Sending contrastive waypoint...");
        geometry_msgs::Point pos;
        pos.x = contrastiveX_;
        pos.y = contrastiveY_;
        pos.z = contrastiveZ_;
        ContrastivePub_.publish(pos);
      }
      // continue
      ros::spinOnce();
      loopRate_.sleep();
    }
  }

  // ros tools
  ros::NodeHandle nodeHandle_;
  ros::Publisher ContrastivePub_;
  ros::Subscriber NavMeshFilteredSub_;
  ros::Rate loopRate_;

  // ratio between frequency and rvizFrequency
  ros::ServiceClient servicePlan_;
  ros::ServiceClient serviceProject_;

  // path planner settings
  double startX_;
  double startY_;
  double startZ_;
  double goalX_;
  double goalY_;
  double goalZ_;
  double contrastiveX_;
  double contrastiveY_;
  double contrastiveZ_;

  // Number of Area Types
  const int noAreaTypes_;
  bool startSet_ = false;
  bool goalSet_ = false;
  bool contrastiveSet_ = false;
  bool updateStart_ = false;
  bool updateGoal_ = false;
  bool updateContrastive_ = false;
  bool updateMesh_ = false; // private flag to check whether a map update required or not

  // Visualization settings
  interactive_markers::MenuHandler::EntryHandle setStartPos_;
  interactive_markers::MenuHandler::EntryHandle setGoalPos_;
  interactive_markers::MenuHandler::EntryHandle setContrastivePos_;

  interactive_markers::MenuHandler menuHandler_;
  interactive_markers::MenuHandler menuHandlerObs_;
  std::vector<visualization_msgs::InteractiveMarker> intMarkerVec_;
  std::vector<visualization_msgs::InteractiveMarkerControl> intControlVec_;
  boost::shared_ptr<interactive_markers::InteractiveMarkerServer> interactiveMarkerServer_;

  visualization_msgs::Marker navMeshFiltered_;
  visualization_msgs::Marker pathList_;
  visualization_msgs::Marker agentStartPos_;
  visualization_msgs::Marker agentGoalPos_;
  std::vector<std_msgs::ColorRGBA> colourList_;
};

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "recast_explanations_interface");
  ros::NodeHandle nodeHandle("~");
  RecastNode rcNode(nodeHandle);

  rcNode.run();
  return 0;
}
