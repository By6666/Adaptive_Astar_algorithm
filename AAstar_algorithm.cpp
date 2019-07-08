#include "include/AAstar_algorithm.h"

//**进行一次总体的A*算法**
//输入：文件序号
//输出：无
void SearchOneMap(int map_num_) {
  //获得map信息
  GrideInput map_info(map_num_);
  map_info.GetOneGrid();
  map_info.PrintMap();  //打印原始map

  //数据传入，构造类对象
  Adaptive_Astar Astar_algorithm(
      map_info.get_grid_rows(), map_info.get_grid_columns(),
      map_info.get_start_pos(), map_info.get_goal_pos(),
      map_info.get_obstacle_pos());

  int search_count = 0;  //搜索计数

  while (1) {
    //规划路径
    std::cout << "**********" << std::endl;
    std::cout << "search num : " << ++search_count << std::endl;
    Astar_algorithm.AstarGetPath();  //以当前起点为起点进行一次路径规划

    if (Astar_algorithm.get_current_path().size() == 0) {
      std::cout << "-——-——-——-——-——-——-——-———-——-——-——-" << std::endl;
      std::cout << "|final result : no path to goal !!|" << std::endl;
      std::cout << "-——-——-——-——-——-——-——-———-——-——-——-" << std::endl;
      break;
    }

    while (!Astar_algorithm.get_current_path().empty()) {
      //更新当前点各临近点的信息
      Astar_algorithm.UpdataMapInfo();

      if (Astar_algorithm.NextStepIsInObstacleList()) {
        break;  //当前点要移动到的下一个点是obstacle
      } else {
        Astar_algorithm.StartMove();
      }
    }

    if (Astar_algorithm.ArriveGoal()) {  //走到了终点
      std::cout << "-——-——-——-——-——-——-——-———-——-——-——-——-" << std::endl;
      std::cout << "|final result: get goal successflly!!|" << std::endl;
      std::cout << "-——-——-——-——-——-——-——-———-——-——-——-——-" << std::endl;
      Astar_algorithm.PrintCountResult();
      break;
    }
  }
}

//**A*算法构造函数**
//输入：地图的行数、列数、起点、终点以、所有的障碍物点
//输出：无
Adaptive_Astar::Adaptive_Astar(int row, int column, Points statr, Points goal,
                               std::vector<Points> obstacle_list_)
    : row_(row),
      column_(column),
      start_pos_(statr),
      goal_pos_(goal),
      map_obstacle_list_(obstacle_list_) {
  current_start_ = start_pos_;
  all_expand_points_count_ = 0;
  search_nums_count_ = 0;
}

//**在一次A*算法中获得当前点的四个临近点**
//输入：当前点的坐标信息
//输出：四个临近点的信息
std::vector<CellInfo> Adaptive_Astar::GetNeighbors(const Points& current_pos) {
  std::vector<CellInfo> neighbors;
  // Up
  if ((current_pos.first - 1) >= 0) {
    neighbors.push_back({0, (goal_pos_.first - current_pos.first) * 0.01,
                         Points(current_pos.first - 1, current_pos.second)});
  }
  // Down
  if ((current_pos.first + 1) < row_) {
    neighbors.push_back({0, -(goal_pos_.first - current_pos.first) * 0.01,
                         Points(current_pos.first + 1, current_pos.second)});
  }
  // Left
  if ((current_pos.second - 1) >= 0) {
    neighbors.push_back({0, -(goal_pos_.second - current_pos.second) * 0.01,
                         Points(current_pos.first, current_pos.second - 1)});
  }
  // Right
  if ((current_pos.second + 1) < column_) {
    neighbors.push_back({0, (goal_pos_.second - current_pos.second) * 0.01,
                         Points(current_pos.first, current_pos.second + 1)});
  }
  return neighbors;
}

//**在整体A*算法中获得当前起点的四个临近点的信息**
//输入：无
//输出：无
void Adaptive_Astar::UpdataMapInfo() {
  if ((current_start_.first - 1) >= 0) {
    if (IsInList(Points(current_start_.first - 1, current_start_.second),
                 map_obstacle_list_))
      current_obstacle_list_.push_back(
          Points(current_start_.first - 1, current_start_.second));
  }
  // Down
  if ((current_start_.first + 1) < row_) {
    if (IsInList(Points(current_start_.first + 1, current_start_.second),
                 map_obstacle_list_))
      current_obstacle_list_.push_back(
          Points(current_start_.first + 1, current_start_.second));
  }
  // Left
  if ((current_start_.second - 1) >= 0) {
    if (IsInList(Points(current_start_.first, current_start_.second - 1),
                 map_obstacle_list_))
      current_obstacle_list_.push_back(
          Points(current_start_.first, current_start_.second - 1));
  }
  // Right
  if ((current_start_.second + 1) < column_) {
    if (IsInList(Points(current_start_.first, current_start_.second + 1),
                 map_obstacle_list_))
      current_obstacle_list_.push_back(
          Points(current_start_.first, current_start_.second + 1));
  }
}

//**以当前点为起点进行一次A*算法的计算**
//输入：无
//输出：无
void Adaptive_Astar::AstarGetPath() {
  {
    std::priority_queue<CellInfo> open_list;  //存放将要遍历的点
    std::vector<Points> close_list(
        current_obstacle_list_);  //存放已经遍历的点以及已知的地图障碍点
    std::map<Points, Points> save_path_hash;  //用于路径回溯
    std::vector<Points> path_result_list;     //存放结果

    CellInfo start_info = {0, 0, current_start_};  //初始化起点的信息
    int search_successful_flg = 1;
    current_expand_points_count_ = 0;
    open_list.push(start_info);  //起点入队列

    while (!open_list.empty()) {
      CellInfo current_cell_pos = open_list.top();
      open_list.pop();
      //找到终点，一次算法结束
      if (current_cell_pos.xoy_ == goal_pos_) {
        //打印最小花费
        last_path_minimal_cost_ = current_cell_pos.g_value_;
        std::cout << "minimal cost :" << current_cell_pos.g_value_ << "  "
                  << current_cell_pos.g_value_ + current_cell_pos.h_value_
                  << std::endl;
        search_successful_flg = 0;  //搜索成功
        break;
      }

      //如果不在closelist中，可以expand
      if (!IsInList(current_cell_pos.xoy_, close_list)) {
        close_list.push_back(current_cell_pos.xoy_);
        //记录扩展点的信息
        current_search_expanded_list_[current_cell_pos.xoy_] =
            current_cell_pos.g_value_;
        ++current_expand_points_count_;

        std::vector<CellInfo> neighbors = GetNeighbors(current_cell_pos.xoy_);

        for (int i = 0; i < neighbors.size(); ++i) {
          if (!IsInList(neighbors[i].xoy_, close_list)) {
            // g(n)
            neighbors[i].g_value_ += current_cell_pos.g_value_ + 1;
            // f(n)=g(n)+h(n)
            // AA*算法对h-value进行迭代
            if (last_search_expanded_list_.find(neighbors[i].xoy_) !=
                last_search_expanded_list_.end()) {
              neighbors[i].h_value_ =
                  last_path_minimal_cost_ -
                  last_search_expanded_list_[neighbors[i].xoy_];
            } else {
              neighbors[i].h_value_ = DistenceToGoal(neighbors[i].xoy_);
            }

            open_list.push(neighbors[i]);
            save_path_hash[neighbors[i].xoy_] = current_cell_pos.xoy_;
          }
        }
      }
    }

    // there is one shortest path to goal
    if (search_successful_flg) {
      std::cout << "search fail !!" << std::endl;
    }
    // nope path to goal
    else {
      std::cout << "search successfully !!" << std::endl;
      Points node = goal_pos_;
      //得到最短路径的坐标向量
      while (node != start_info.xoy_) {
        path_result_list.push_back(node);
        node = save_path_hash[node];
      }
    }

    current_path_.clear();
    current_path_ = path_result_list;

    ++search_nums_count_;
    all_expand_points_count_ += current_expand_points_count_;  // expand计数累加

     SearchResultPrint();  //打印一次搜索结果

    //迭代赋值
    last_search_expanded_list_.clear();
    last_search_expanded_list_ = current_search_expanded_list_;
    current_search_expanded_list_.clear();
  }
}
//**打印一次搜索结果**
//输入：无
//输出：无
void Adaptive_Astar::SearchResultPrint() {
  for (int i = 0; i < row_; ++i) {
    for (int j = 0; j < column_; ++j) {
      if (current_start_.first == i && current_start_.second == j)
        std::cout << "s ";

      else if (goal_pos_.first == i && goal_pos_.second == j)
        std::cout << "g ";

      else if (IsInList(Points(i, j), current_obstacle_list_))
        std::cout << "x ";

      else if (IsInList(Points(i, j), current_path_))
        std::cout << "o ";
      else
        std::cout << "_ ";
    }
    std::cout << std::endl;
  }
  std::cout << "shortest path step nums : " << current_path_.size()
            << "    expand point nums : " << current_expand_points_count_
            << std::endl;

  std::cout << std::endl << std::endl;
}

//**打印计数结果**
//输入：无
//输出：无
void Adaptive_Astar::PrintCountResult() {
  std::cout << std::endl
            << "The nums of search : " << search_nums_count_
            << "  total expanded nums : " << all_expand_points_count_
            << std::endl
            << std::endl;
}