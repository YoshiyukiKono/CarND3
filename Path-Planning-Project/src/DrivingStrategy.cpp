#include "DrivingStrategy.h"


void DrivingStrategyMiddle::adopt(BehaviorPlanner* behavior_planner){
	behavior_planner->init_recommend_lane();
	bool is_safe_to_move_left = behavior_planner->is_safe_to_move(behavior_planner->cur_lane - 1);
	bool is_safe_to_move_right = behavior_planner->is_safe_to_move(behavior_planner->cur_lane + 1);

	if (is_safe_to_move_right && is_safe_to_move_left) {
std::cout << "adopt - L & R are SAFE. So, call recommend_lane"  << std::endl;
		behavior_planner->lane_change = true;
		behavior_planner->target_lane = behavior_planner->recommend_lane();
std::cout << "adopt - target_lane:" <<  behavior_planner->target_lane << std::endl;
	} else if (is_safe_to_move_right) {

		behavior_planner->lane_change = true;
		behavior_planner->target_lane = behavior_planner->cur_lane + 1;
std::cout << "adopt - only Right is safe. target_lane:" <<  behavior_planner->target_lane << std::endl;
	} else if (is_safe_to_move_left) {

		behavior_planner->lane_change = true;
		behavior_planner->target_lane = behavior_planner->cur_lane - 1;
std::cout << "adopt - only Left is safe. target_lane:" <<  behavior_planner->target_lane  << std::endl;
	}
}
void DrivingStrategyLeftEnd::adopt(BehaviorPlanner* behavior_planner){
	if (behavior_planner->is_safe_to_move(behavior_planner->cur_lane + 1)) {
		behavior_planner->lane_change = true;
		behavior_planner->target_lane = behavior_planner->cur_lane + 1;
	}
}
void DrivingStrategyRightEnd::adopt(BehaviorPlanner* behavior_planner){
	if (behavior_planner->is_safe_to_move(behavior_planner->cur_lane - 1)) {
		behavior_planner->lane_change = true;
		behavior_planner->target_lane = behavior_planner->cur_lane - 1;
	}
}