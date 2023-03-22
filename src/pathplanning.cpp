#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PointStamped.h"
#include "visualization_msgs/Marker.h"
#include "turtlebot_test/NAV_GOAL.h"
#include "LinkedList.h"
#include "tf/tf.h"
#include "Map.h"
#include <chrono>
#include <thread>

//globals
ros::Publisher pub;
ros::Publisher vis_pub; 
MapPoint* current_goal=nullptr;
//map data
nav_msgs::OccupancyGrid currentMap;
nav_msgs::Odometry currentPose;
Linkedlist<MapPoint*> currentPath;
void drawRedPoint(MapPoint* current){
    visualization_msgs::Marker marker;
        marker.header.frame_id = "base_link";
        marker.header.stamp = ros::Time();
        marker.id = 0;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = current->x_location;
        marker.pose.position.y = current->y_location;
        marker.pose.position.z = 1;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        vis_pub.publish(marker);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
}
void drawGreenPoint(MapPoint* current){
    visualization_msgs::Marker marker;
        marker.header.frame_id = "base_link";
        marker.header.stamp = ros::Time();
        marker.id = 0;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = current->x_location;
        marker.pose.position.y = current->y_location;
        marker.pose.position.z = 1;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        vis_pub.publish(marker);
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
}
void mappingCallback(const nav_msgs::OccupancyGrid::ConstPtr& input){
    ROS_INFO("recieved map message");
    // tf::Quaternion q(input->info.origin.orientation.x, input->info.origin.orientation.y, input->info.origin.orientation.z, input->info.origin.orientation.w);
    // tf::Matrix3x3 m(q);
    // double roll, pitch, yaw;
    // m.getRPY(roll, pitch, yaw);
    // ROS_INFO("Map rotation: %f", yaw);
    memcpy(&currentMap, input.get(), sizeof(nav_msgs::OccupancyGrid));
    const Map *map = Map::GetInstance(input);
}
void odometryCallback(const nav_msgs::Odometry::ConstPtr& input)
{
  memcpy(&currentPose, input.get(), sizeof(nav_msgs::Odometry));
}
void publishMarkers(){
    for(int i=0; i<currentPath.length; i++){
        MapPoint* current = currentPath.get(i)->data;
        visualization_msgs::Marker marker;
        marker.header.frame_id = "base_link";
        marker.header.stamp = ros::Time();
        marker.id = 0;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = current->x_location;
        marker.pose.position.y = current->y_location;
        marker.pose.position.z = 1;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        vis_pub.publish(marker);
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}
Linkedlist<MapPoint*> reconstructPath(MapPoint* start, MapPoint* end, map_point_unordered_map cameFrom)
{
  Linkedlist<MapPoint*> path;
  path.insertLast(end);
  while(!(cameFrom.find(end)==cameFrom.end()))
  {
    end = cameFrom[end];
    path.insertFirst(end);
  }
  return path;
}
//A* implementation
Linkedlist<MapPoint*> findPath(MapPoint* start, MapPoint* end)
    {
        map_point_unordered_map cameFrom;
        map_point_priority_queue openSet;
        MapPoint* current = start;
        //If starting location is the goal, the path will be just the starting node
        if(current==end){
            return reconstructPath(start, end, cameFrom);
        }
        else{
            openSet.emplace(current);
        }
        current->g=0;
        current->setH(current);
        current->f=distance(current->x_index, current->y_index, end->x_index, end->y_index);
        while(!(current==end) && !openSet.size()==0) {
            //DangerPoint oldCurrent = current;
            current = openSet.top();
            //drawRedPoint(current);
            openSet.pop();
            //oldCurrent.closed = true;
            std::vector<MapPoint*> neighbors = current->neighbors;
            for(MapPoint* n: neighbors) {
                double tentative_gscore = current->g+distance(n->x_index, n->y_index, current->x_index, current->y_index);
                if(tentative_gscore<n->g){
                    cameFrom[n]=current;
                    n->g = tentative_gscore;
                    n->f = tentative_gscore+distance(n->x_index, n->y_index, end->x_index, end->y_index);
                    openSet.remove(n);
                    openSet.emplace(n);
                    drawRedPoint(n);
                }
            }

        }
        //reconstruct the path to the node starting from the goal node to the beginning
        return reconstructPath(start, end, cameFrom);
    }
void setAndFindPath(Map* map){
    if(map==nullptr){
        return;
    }
    if(current_goal==nullptr){
        return;
    }
    ROS_INFO("Finding path to point %f, %f from %f, %f", current_goal->x_location, current_goal->y_location, currentPose.pose.pose.position.x, currentPose.pose.pose.position.y);
    map->resetPoints();
    MapPoint* start = map->getClosest(currentPose.pose.pose.position.x, currentPose.pose.pose.position.y);
    currentPath = findPath(start, current_goal);
    MapPoint* intermediate_nav_target = nullptr;
    if(currentPath.length>=5){
        intermediate_nav_target = currentPath.get(4)->data;
    } else if (currentPath.length<5&&currentPath.length>0){
        intermediate_nav_target = currentPath.get(currentPath.length)->data;
    } else{
        ROS_ERROR("Error: Path length is 0!");
    }
    turtlebot_test::NAV_GOAL goal;
    goal.x=intermediate_nav_target->x_location;
    goal.y=intermediate_nav_target->x_location;
    publishMarkers();
    pub.publish(goal);
}
void clickCallback(const geometry_msgs::PointStamped::ConstPtr& input){
    double target_x = input.get()->point.x;
    double target_y = input.get()->point.y;
    //turtlebot_test::NAV_GOAL goal;
    //goal.x = target_x;
    //goal.y=target_y;
    //pub.publish(goal);
    Map* map = Map::GetInstance();
    current_goal = map->getClosest(target_x, target_y);
    setAndFindPath(map);
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "pathplanning");
  ros::NodeHandle n;
  ros::Subscriber mapsub = n.subscribe("/map", 1, mappingCallback);
  ros::Subscriber clicksub = n.subscribe("/clicked_point", 1, clickCallback);
  ros::Subscriber odomsub = n.subscribe("/odom", 1, odometryCallback);

  pub = n.advertise<turtlebot_test::NAV_GOAL>("NAV_GOAL", 1);
  vis_pub = n.advertise<visualization_msgs::Marker>( "/visualization_marker", 0 );
  currentMap = {};
  //set update rate to 1Hz
  ros::Rate loop_rate(1);
  while(ros::ok()){
    Map* map = Map::GetInstance();
    setAndFindPath(map);
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}