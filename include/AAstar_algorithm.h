#ifndef PLANNER_AASTAR_ALGORITHM_H
#define PLANNER_AASTAR_ALGORITHM_H

#include <algorithm>
#include <cmath>
#include <iostream>
#include <map>
#include <queue>
#include <vector>

#include "grid_input.h"

void SearchOneMap(int map_num_);
void PrintSumResult();            //打印结果统计

//每一个cell的信息，使用结构体定义
struct CellInfo {
  double h_value_;
  double g_value_;
  Points xoy_;
  bool operator<(const CellInfo& pos) const {
    // if f-vaule 相同，比较 g-vaule
    if ((h_value_ + g_value_) == (pos.h_value_ + pos.g_value_))
      return g_value_ < pos.g_value_;
    else
      return (h_value_ + g_value_) > (pos.h_value_ + pos.g_value_);
  }
};

class Adaptive_Astar {
 public:
  Adaptive_Astar(int row_, int columns_, Points statr_, Points goal_,
                 std::vector<Points> obstacle_list_);  //构造函数
  Adaptive_Astar(const Adaptive_Astar& as) = delete;  //不使用复制构造函数

  //******下面函数用于一次A*规划算法中*******//
  void AstarGetPath();       // A*算法函数
  void SearchResultPrint();  //打印搜索结果
                             //获得当前节点的neighbors
  std::vector<CellInfo> GetNeighbors(const Points& current_pos);
  //计算当前点与终点距离的函数
  inline double DistenceToGoal(const Points& current) {
    //曼哈顿距离
    return fabs(current.first - goal_pos_.first) +
           fabs(current.second - goal_pos_.second);
  }
  //**************************************//

  //**********下面函数用于整体算法中**********//
  //获得当前节点的neighbors，但仅仅包含障碍信息
  void UpdataMapInfo();

  void StartMove() {
    current_start_ = current_path_.back();
    current_path_.pop_back();
    ++move_step_nums_;
  }

  bool ArriveGoal() { return current_start_ == goal_pos_; }
  inline bool NextStepIsInObstacleList() {
    return IsInList(current_path_.back(), current_obstacle_list_);
  }
  //打印计数结果
  void PrintCountResult();
  //**************************************//

  //**********类内私有成员赋值函数**********//
  inline int& set_row() { return row_; }
  inline int& set_column() { return column_; }
  inline Points& set_start_pos() { return start_pos_; }
  inline Points& set_goal_pos() { return goal_pos_; }
  inline Points& set_current_start() { return current_start_; }
  inline std::vector<Points>& set_map_obstacle_list() {
    return map_obstacle_list_;
  }
  inline std::vector<Points>& set_current_obstacle_list() {
    return current_obstacle_list_;
  }
  inline std::vector<Points>& set_current_path() { return current_path_; }
  //************************************//

  //**********获取类内私有成员函数**********//
  inline int get_row() const { return row_; }
  inline int get_column() const { return column_; }
  inline Points get_start_pos() const { return start_pos_; }
  inline Points get_goal_pos() const { return goal_pos_; }
  inline Points get_current_start() const { return current_start_; }
  inline std::vector<Points> get_map_obstacle_list() const {
    return map_obstacle_list_;
  }
  inline std::vector<Points> get_current_obstacle_list() const {
    return current_obstacle_list_;
  }
  inline std::vector<Points> get_current_path() const { return current_path_; }
  inline int get_all_expand_nums() const { return all_expand_points_count_; }
  inline int get_search_nums() const { return search_nums_count_; }
  inline int get_move_step_nums() const { return move_step_nums_; }
  //************************************//

 private:
  int row_, column_;
  Points start_pos_, goal_pos_;
  Points current_start_;                   //当前起点
  std::vector<Points> map_obstacle_list_;  //所有障碍物list
  std::vector<Points> current_path_;       //当前路径
  std::vector<Points> current_obstacle_list_;
  int current_expand_points_count_,  //一次算法中的expand计数
      all_expand_points_count_,      //整体算法中的expand计数
      search_nums_count_,            //搜索次数计数
      move_step_nums_;                         //移动步数

  // for Adaptive A* algorithm
  double last_path_minimal_cost_;  //上一条路径中的最小花费
  // map<(点的坐标),(上一次的g-value)>
  std::map<Points, double> last_search_expanded_list_;  //记录上一次扩展点的信息
  std::map<Points, double>
      current_search_expanded_list_;  //记录上一次扩展点的信息

  //判断点是否在list中
  inline bool IsInList(const Points& point, const std::vector<Points>& list) {
    return std::find(list.begin(), list.end(), point) != list.end();
  }
};

#endif