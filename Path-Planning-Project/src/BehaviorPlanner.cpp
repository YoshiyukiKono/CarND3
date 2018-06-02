#include <iostream>
#include <math.h>
#include <map>
#include <vector>
#include <string>
#include <iterator>
#include <memory>

#include "BehaviorPlanner.h"
#include "vehicle.h"
#include "DrivingStrategy.h"

/**
 * Initializes BehaviorPlanner
 */
BehaviorPlanner::BehaviorPlanner() {
}

BehaviorPlanner::~BehaviorPlanner() {}

// |----|----|----|
int BehaviorPlanner::convert_to_lane(float d) {
	int lane = 0;
	if(d < 4) //x < 4
	{
		lane = 0;
	}
	else if(d <= 8 && d >= 4) // 0 < x < 4 ;4 < x < 8; 8 < x < 12
	{
		lane = 1;
	} else if(d > 8) // 8 < x
	{
		lane = 2;
	}
	return lane;
}
void BehaviorPlanner::populate_traffic(vector<vector<double>>  sensor_fusion) {

	faster_lane_speed = 0;
	farther_lane_s = 0;
	this->vehicles.clear();

  	for(int i = 0; i < sensor_fusion.size(); i++)
  	{
  		float d = sensor_fusion[i][6];
  		int lane = convert_to_lane(d);

		double vx = sensor_fusion[i][3];
		double vy = sensor_fusion[i][4];
		double check_speed = sqrt(vx*vx + vy*vy);
		double check_car_s = sensor_fusion[i][5];
		double a = 0;

		Vehicle vehicle = Vehicle(lane,check_car_s,check_speed,a);
		this->vehicles.insert(std::pair<int,Vehicle>(sensor_fusion[i][0],vehicle));
		
  	}
}

bool BehaviorPlanner::is_safe_to_move(int lane_num) {

	bool is_safe = false;
	bool is_fore_safe = false;
	bool is_back_safe = false;
	int back_vehicle_s = 0;
	int back_vehicle_v = 0;
	int fore_vehicle_s = 0;
	int fore_vehicle_v = 0;

	for(map<int, Vehicle>::iterator it = this->vehicles.begin();
		it != this->vehicles.end();
		it++)
	{
		int v_id = it->first;
		Vehicle v = it->second;

		if(v.lane == 0) {
			L_vehicle_num += 1;
		}
		if(v.lane == 2) {
			R_vehicle_num += 1;
		}
		if(v.lane == lane_num) {
			if (v.s >= ego_vehicle.s) { // fore vehicles
				if (fore_vehicle_v == 0) { // not yet initialized
					fore_vehicle_s = v.s;
					fore_vehicle_v = v.v;
				} else{
					if(fore_vehicle_s > v.s) { // check the nearest one
						fore_vehicle_s = v.s;
						fore_vehicle_v = v.v;
					}
				}
			} else { // back vehicles
				if (back_vehicle_v == 0) {
					back_vehicle_s = v.s;
					back_vehicle_v = v.v;
				} else{
					if(back_vehicle_s < v.s) { // check the nearest one
						back_vehicle_s = v.s;
						back_vehicle_v = v.v;
					}
				}
			}
		}
	}
	if (back_vehicle_s == 0 || (ego_vehicle.s - back_vehicle_s) > SAFE_DISTANCE) {
		is_back_safe = true;
	} else {
std::cout << "(lane:" << lane_num << ")[Back is not safe.] back_vehicle_s:" << back_vehicle_s << " ego_vehicle.s:" << ego_vehicle.s << std::endl;
	}

    if(fore_vehicle_v == 0 || (fore_vehicle_s - ego_vehicle.s) > SAFE_DISTANCE) {
			is_fore_safe = true;
	} else {
std::cout << "(lane:" << lane_num << ")[Fore is not safe.]  fore_vehicle_s:" << fore_vehicle_s  << " ego_vehicle.s:" << ego_vehicle.s << std::endl;
	}

	if (is_fore_safe && is_back_safe) {
		is_safe = true;
	}

	if(lane_num == 0) {
		farther_L_s = fore_vehicle_s;
		faster_L_v = fore_vehicle_v;
	}
	if(lane_num == 2) {
		farther_R_s = fore_vehicle_s;
		faster_R_v = fore_vehicle_v;
	}
    return is_safe;
}

void BehaviorPlanner::add_ego(int lane_num, int s, double speed) {
	ego_vehicle = Vehicle(lane_num, s, speed, 0);
}

void BehaviorPlanner::init_recommend_lane() {
std::cout << "init_recommend_lane:" << std::endl;

	recommended_lane = -1;
	farther_lane_s = 0;
	faster_lane_speed = 0;
	farther_L_s  = 0;
	faster_L_v = 0;
	farther_R_s = 0;
	faster_R_v = 0;
	
	
	L_vehicle_num = 0;
	R_vehicle_num = 0;
}

int BehaviorPlanner::recommend_lane() {

	if (farther_L_s == 0) {
		recommended_lane = 0;
	} else if (farther_R_s == 0) {
		recommended_lane = 2;
	} else if (farther_L_s > farther_R_s) {
		recommended_lane = 0;
	} else {
		recommended_lane = 2;
	}
std::cout << "recommend_lane - recommended_lane:" << recommended_lane << " farther_lane_s:" << farther_lane_s << " faster_lane_speed:" << faster_lane_speed << std::endl;
std::cout << "recommend_lane - [Right] farther_R_s:" << farther_R_s << " faster_R_v:" << faster_R_v << std::endl;
std::cout << "recommend_lane - [Left]  farther_L_s:" << farther_L_s << " faster_L_v:" << faster_L_v << std::endl;
std::cout << "recommend_lane - [Num of Cars]  L_vehicle_num:" << L_vehicle_num << " R_vehicle_num:" << R_vehicle_num << std::endl;
std::cout << "recommend_lane - [Ego]  ego_vehicle.s:" << ego_vehicle.s << " ego_vehicle.v:" << ego_vehicle.v << std::endl;
	if (farther_R_s == 0 || farther_L_s == 0) {
	    for(map<int, Vehicle>::iterator it = this->vehicles.begin();
	    	it != this->vehicles.end();
	    	it++)
	    {
	    	int v_id = it->first;
	        Vehicle v = it->second;
std::cout << "recommend_lane - v_id:" << v_id << " v.lane:" << v.lane << " v.s:" << v.s <<  " v.v:" << v.v << std::endl;
		}
	}
	return recommended_lane;
}

void BehaviorPlanner::plan(int& lane, int& prev_size) {

	bool lane_change = false;
	too_close = false;
	ego_vehicle.target_speed = 0;
	
	this->cur_lane = lane;
	this->target_lane = lane;
	
	for(map<int, Vehicle>::iterator it = this->vehicles.begin();
		it != this->vehicles.end();
		it++)
	{
		int v_id = it->first;
		Vehicle v = it->second;

		if (v.lane == ego_vehicle.lane) {

			double check_speed = v.v;
			double check_car_s = v.s;

			// Use to compare with the other lane ???
			ego_lane_speed = check_speed;
			//ego_lane_s = check_car_s;
			
			check_car_s += ((double)prev_size*.02*check_speed); // if using previous points can project s value out...
			// check s values greater than mine and s gap
			const int SAFE_DISTANCE = 30;
			if((check_car_s > ego_vehicle.s) && ((check_car_s - ego_vehicle.s) < SAFE_DISTANCE))//30))
			{
				// Do some logic here, lower reference velocity so we dont crash into the car in front of us, could...
				// also flog to try to change lanes.
				//ref_vel = 29.5;//mph
				too_close = true;
				const int DANGER_DISTANCE = 5;
				if (check_car_s - ego_vehicle.s <= DANGER_DISTANCE) {
					ego_vehicle.target_speed = check_speed;
				}

				drive_strategies[lane]->adopt(this);
			}
		}
	}

	if(!lane_change && is_better_to_move())
	{
		target_lane = recommend_lane_in_a_long_run();
		lane_change = true;
	}

	avoid_if_dangerous_situation();
}


bool BehaviorPlanner::is_initialized() {
	return drive_strategies.size() == 0;
}
void BehaviorPlanner::update_route_planning_offline() {

	drive_strategies[0] = shared_ptr<DrivingStrategyLeftEnd>(new DrivingStrategyLeftEnd());
	drive_strategies[1] = shared_ptr<DrivingStrategyMiddle>(new DrivingStrategyMiddle());
	drive_strategies[2] = shared_ptr<DrivingStrategyRightEnd>(new DrivingStrategyRightEnd());

}
void BehaviorPlanner::train(vector<vector<double>> sensor_fusion) {

	save_previous_state();

	for(int i = 0; i < sensor_fusion.size(); i++)
	{
		float d = sensor_fusion[i][6];
		int lane = convert_to_lane(d);
		double s = sensor_fusion[i][5];

		int vehicle_id = sensor_fusion[i][0];
		int prev_lane = get_prev_lane(vehicle_id);
		float s_dot;
		float d_dot;
		update_prediction_model(lane, prev_lane, s, d, s_dot, d_dot);
	}
}
void BehaviorPlanner::save_previous_state() {
	// save the previous state to generate the label for supervised learning by comparing with the future state.
}
int BehaviorPlanner::get_prev_lane(int vehicle_id) {
	// retrieve the previous lane from the historical data
	return 0;
}
void BehaviorPlanner::update_prediction_model(int lane, int prev_lane, float s, float d, float s_dot, float d_dot) {
	// may have line-wise behavioral model
}
void BehaviorPlanner::predict() {
	for(map<int, Vehicle>::iterator it = this->vehicles.begin();
		it != this->vehicles.end();
		it++)
	{
		predict_vehicle(it->second);
	}
}
void BehaviorPlanner::predict_vehicle(Vehicle& vehicle) {
	// apply prediction model
}

bool BehaviorPlanner::is_better_to_move() {
	// Take into account the following points
	// - overall traffic situation, for example, the cars may run faster in the other lane.
	// - predicted future state
	//
	// Then decide if the ego car should move or not, for example
	// - Move to the other lane where cars run smoothly
	// - Move to middle lane for the flexibility in the future unless middle lane's condition is worth 
	return false;
}
int BehaviorPlanner::recommend_lane_in_a_long_run() {
	// recommend lane by calculating the cost to move
	// Maybe, it is better to plan the route rather than recommending the lane at the moment.
	return 0;
}
void BehaviorPlanner::avoid_if_dangerous_situation() {
	// check the distance to the cars running side lanes
	// if there is any car that runs in a dangerous way, 
	// the ego car will drop its speed or move to the other lane as long as it is safe.
}