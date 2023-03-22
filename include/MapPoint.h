#pragma once
#include <stdint.h>
#include <vector>
#include <cmath>
#include <unordered_map>
#include <queue>
#define BLOCKED_WEIGHT 100;
//map point struct
class MapPoint{
public:
  MapPoint(int x, int y){
    x_index=x;
    y_index=y;
  }
  MapPoint(){
  }
  //location
  int x_index=0;
  int y_index=0;
  double x_location=0;
  double y_location=0;
  //probability blocked
  uint8_t data=0;
  //mapping data
  std::vector<MapPoint*> neighbors = {};
  //A* heuristics
  double f=DBL_MAX;
  double g=DBL_MAX;
  double h=0;
  void setF(){
    f=g+h;
  }
  void setG(MapPoint* n){
    g = n->g+sqrt((x_index-n->x_index)*(x_index-n->x_index)+(y_index-n->y_index)*(y_index-n->y_index))/*+data*BLOCKED_WEIGHT*/;
  }
  void setH(MapPoint* n){
    h = sqrt((x_index-n->x_index)*(x_index-n->x_index)+(y_index-n->y_index)*(y_index-n->y_index));
  }
  //operators
  bool operator==(const MapPoint &p) const {
        return x_index == p.x_index && y_index == p.y_index;
  }
};
//hash function for MapPoint struct
struct map_point_hash_fn
{
    std::size_t operator() (const MapPoint* point) const
    {
        std::size_t h1 = std::hash<int>()(point->x_index);
        std::size_t h2 = std::hash<int>()(point->y_index);
 
        return h1 ^ h2;
    }
};
//compare function for MapPoint struct
struct map_point_compare_fn
{
    bool operator() (MapPoint* a, MapPoint* b) const
    {
        bool test = a->f>b->f;
        return test;
    }
};
template<typename T, typename A, typename B>
class custom_priority_queue : public std::priority_queue<T, A, B>
{
  public:

      bool remove(const T& value) {
          auto it = std::find(this->c.begin(), this->c.end(), value);
       
          if (it == this->c.end()) {
              return false;
          }
          if (it == this->c.begin()) {
              // deque the top element
              this->pop();
          }    
          else {
              // remove element and re-heap
              this->c.erase(it);
              std::make_heap(this->c.begin(), this->c.end(), this->comp);
         }
         return true;
     }
};
using map_point_unordered_map = std::unordered_map<MapPoint*, MapPoint*, map_point_hash_fn>;
using map_point_priority_queue = custom_priority_queue<MapPoint*, std::vector<MapPoint*>, map_point_compare_fn>;