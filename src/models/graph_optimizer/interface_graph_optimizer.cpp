/**
 * @file interface_graph_optimizer.cpp
 * @brief interface_graph_optimizer的实现
 * @author Ren Qian
 * @version 0.1
 * @date 2020-03-13 21:53:02
 */
#include "lidar_localization/models/graph_optimizer/interface_graph_optimizer.hpp"

namespace lidar_localization {
void InterfaceGraphOptimizer::SetMaxIterationsNum(int max_iterations_num) {
    max_iterations_num_ = max_iterations_num;
}
}
