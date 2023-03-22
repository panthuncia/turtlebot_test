#pragma once
#include <mutex>
#include <string>
#include <vector>
#include "nav_msgs/OccupancyGrid.h"
#include "MapPoint.h"
double distance(double x1, double y1, double x2, double y2)
{
    float x2x1 = x2-x1;
    float y2y1 = y2-y1;
    return sqrt(x2x1*x2x1 + y2y1*y2y1);
}
uint8_t getElement(const nav_msgs::OccupancyGrid::ConstPtr& grid, unsigned int x, unsigned int y)
{
    //C++ standard constrains vectors to contiguous memory, so this hack works
    return ((uint8_t *) &(grid.get()->data[0]))[x*grid.get()->info.width + y];
}
/**
 * Thread-safe singleton implementation
 */
class Map
{

    /**
     * The Singleton's constructor/destructor should always be private to
     * prevent direct construction/desctruction calls with the `new`/`delete`
     * operator.
     */
private:
    static Map * pinstance_;
    static std::mutex mutex_;

protected:
    Map(const nav_msgs::OccupancyGrid::ConstPtr& value)
    {
        InitMap(value);
    }
    //destructor frees map memory
    ~Map() {
        for(std::vector<MapPoint*> outer : _map){
            for(MapPoint* point : outer){
                delete point;
            }
        }
    }
    void initNeighbors(){
        //int executions = 0;
        for(std::vector<MapPoint*> outer : _map){
            for(MapPoint* point : outer){
                // executions++;
                // if(executions==147073){
                //     executions++;
                // }
                //ROS_INFO("%i, X: %i  Y: %i ",executions, point->x_index, point->y_index);
                if(point->y_index-1>=0)
                    point->neighbors.push_back(_map[point->x_index][point->y_index-1]);
                if(point->x_index-1>=0)
                    point->neighbors.push_back(_map[point->x_index-1][point->y_index]);
                if(point->x_index+1<X)
                    point->neighbors.push_back(_map[point->x_index+1][point->y_index]);
                if(point->y_index+1<Y)
                    point->neighbors.push_back(_map[point->x_index][point->y_index+1]);
                if(point->x_index-1>=0&&point->y_index-1>=0)
			        point->neighbors.push_back(_map[point->x_index-1][point->y_index-1]);
		        if(point->x_index+1<X&&point->y_index-1>=0)
			        point->neighbors.push_back(_map[point->x_index+1][point->y_index-1]);
		        if(point->x_index-1>=0&&point->y_index+1<Y)
			        point->neighbors.push_back(_map[point->x_index-1][point->y_index+1]);
		        if(point->x_index+1<X&&point->y_index+1<Y)
			        point->neighbors.push_back(_map[point->x_index+1][point->y_index+1]);
            }
        }
    }
    std::vector<std::vector<MapPoint*>> _map = {};
    int X, Y;
public:
    /**
     * Singletons should not be cloneable.
     */
    Map(Map &other) = delete;
    /**
     * Singletons should not be assignable.
     */
    void operator=(const Map &) = delete;
    /**
     * This is the static method that controls the access to the singleton
     * instance. On the first run, it creates a singleton object and places it
     * into the static field. On subsequent runs, it returns the client existing
     * object stored in the static field.
     */

    static Map *GetInstance(const nav_msgs::OccupancyGrid::ConstPtr& value);
    static Map *GetInstance();
    void resetPoints(){
        for(std::vector<MapPoint*> outer : _map){
            for(MapPoint* point : outer){
                point->g=DBL_MAX;
                point->f=DBL_MAX;
            }
        }
    }
    /**
     * Finally, any singleton should define some business logic, which can be
     * executed on its instance.
     */
    void InitMap(const nav_msgs::OccupancyGrid::ConstPtr& value)
    {
        //free any existing allocated memory
        for(std::vector<MapPoint*> outer : _map){
            for(MapPoint* point : outer){
                delete point;
            }
        }
        ROS_INFO("freed memory");
        X=value.get()->info.height;
        Y=value.get()->info.width;
        _map.resize(X);
        for(int i=0; i<X; i++){
            _map[i].resize(Y);
            for(int j=0; j<Y; j++){
                MapPoint* newPoint = new MapPoint();
                newPoint->x_index=i;
                newPoint->y_index=j;
                double world_x = i*value.get()->info.resolution+value.get()->info.origin.position.x;
                double world_y = j*value.get()->info.resolution+value.get()->info.origin.position.y;
                newPoint->x_location = world_x;
                newPoint->y_location = world_y;
                newPoint->data=getElement(value, i, j);
                _map[i][j]=newPoint;
            }
        }
        ROS_INFO("added nodes");
        //init neighbor connections
        initNeighbors();
        ROS_INFO("added neighbors");
    }
    //updates occupancy grid
    //assumes map dimensions are static
    void UpdateMap(const nav_msgs::OccupancyGrid::ConstPtr& value){
        for(int i=0; i<X; i++){
            for(int j=0; j<Y; j++){
                _map[i][j]->data=getElement(value, i, j);
            }
        }
    }
    //TODO this can be optimized
    MapPoint* getClosest(double x, double y){
        double closestDist = DBL_MAX;
        MapPoint* closestPoint;
        for(std::vector<MapPoint*> outer : _map){
            for(MapPoint* point : outer){
                double dist = distance(x, y, point->x_location, point->y_location);
                if(dist<closestDist){
                    closestDist = dist;
                    closestPoint = point;
                }
            }
        }
        return closestPoint;
    }
    std::vector<std::vector<MapPoint*>> value() const{
        return _map;
    } 
};

/**
 * Static methods should be defined outside the class.
 */

Map* Map::pinstance_{nullptr};
std::mutex Map::mutex_;

/**
 * The first time we call GetInstance we will lock the storage location
 *      and then we make sure again that the variable is null and then we
 *      set the value.
 */
Map *Map::GetInstance(const nav_msgs::OccupancyGrid::ConstPtr& value)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (pinstance_ == nullptr)
    {
        ROS_INFO("creating new map");
        pinstance_ = new Map(value);
    } else{
        pinstance_->UpdateMap(value);
    }
    return pinstance_;
}
Map *Map::GetInstance()
{
    return pinstance_;
}