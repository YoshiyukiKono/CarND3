#include <iostream>
#include "BehaviorPlanner.h"
#include "vehicle.h"
#include <math.h>
#include <map>
#include <string>
#include <iterator>

// 5/4 12:13 âEÇ…ã»Ç™ÇÁÇ»Ç¢ÅIÅI
/**
 * Initializes BehaviorPlanner
 */
BehaviorPlanner::BehaviorPlanner(int speed_limit, double traffic_density, vector<int> lane_speeds) {

    this->num_lanes = lane_speeds.size();
    this->lane_speeds = lane_speeds;
    this->speed_limit = speed_limit;
    this->density = traffic_density;
    this->camera_center = this->update_width/2;

}

BehaviorPlanner::BehaviorPlanner() {
	faster_lane_speed = 0;
	farther_lane_s = 0;
}

BehaviorPlanner::~BehaviorPlanner() {}

Vehicle BehaviorPlanner::get_ego() {
	
	return this->vehicles.find(this->ego_key)->second;
}
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
  	// find ref_v to use
  	for(int i = 0; i < sensor_fusion.size(); i++)
  	{
  		// car is my lane
  		float d = sensor_fusion[i][6];
  		int lane = convert_to_lane(d);

		double vx = sensor_fusion[i][3];
		double vy = sensor_fusion[i][4];
		double check_speed = sqrt(vx*vx + vy*vy);
		double check_car_s = sensor_fusion[i][5];
		double a = 0;
		// int lane, float s, float v, float a, string state
		Vehicle vehicle = Vehicle(lane,check_car_s,check_speed,a);
		this->vehicles.insert(std::pair<int,Vehicle>(sensor_fusion[i][0],vehicle));
		
  	}
}

void BehaviorPlanner::populate_traffic() {
	
	int start_s = max(this->camera_center - (this->update_width/2), 0);
	for (int l = 0; l < this->num_lanes; l++)
	{
		int lane_speed = this->lane_speeds[l];
		bool vehicle_just_added = false;
		for(int s = start_s; s < start_s+this->update_width; s++)
		{
			
			if(vehicle_just_added)
			{
				vehicle_just_added = false;
			}
			if(((double) rand() / (RAND_MAX)) < this->density)
			{
				
				Vehicle vehicle = Vehicle(l,s,lane_speed,0);
				vehicle.state = "CS";
				this->vehicles_added += 1;
				this->vehicles.insert(std::pair<int,Vehicle>(vehicles_added,vehicle));
				vehicle_just_added = true;
			}
		}
	}
	
}
/*
void BehaviorPlanner::advance() {
	
	map<int ,vector<Vehicle> > predictions;

	map<int, Vehicle>::iterator it = this->vehicles.begin();
    while(it != this->vehicles.end())
    {
        int v_id = it->first;
        vector<Vehicle> preds = it->second.generate_predictions();
        predictions[v_id] = preds;
        it++;
    }
	it = this->vehicles.begin();
	while(it != this->vehicles.end())
    {
    	int v_id = it->first;
        if(v_id == ego_key)
        {   
        	vector<Vehicle> trajectory = it->second.choose_next_state(predictions);
        	it->second.realize_next_state(trajectory);
        }
        else {
            it->second.increment(1);
        }
        it++;
    }
    
}
*/
void BehaviorPlanner::find_vehicle_forward(int lane_num, int s) {
	
	map<int, Vehicle>::iterator it = this->vehicles.begin();
    while(it != this->vehicles.end())
    {
    	int v_id = it->first;
        Vehicle v = it->second;
        if(v.lane == lane_num && v.s == s)
        {
        	this->vehicles.erase(v_id);
        }
        it++;
    }
}

void BehaviorPlanner::find_vehicle_backward(int lane_num, int s) {
	
	map<int, Vehicle>::iterator it = this->vehicles.begin();
    while(it != this->vehicles.end())
    {
    	int v_id = it->first;
        Vehicle v = it->second;
        if(v.lane == lane_num && v.s == s)
        {
        	this->vehicles.erase(v_id);
        }
        it++;
    }
    
}

void BehaviorPlanner::add_ego(int lane_num, int s, vector<int> config_data) {
	
	map<int, Vehicle>::iterator it = this->vehicles.begin();
    while(it != this->vehicles.end())
    {
    	int v_id = it->first;
        Vehicle v = it->second;
        if(v.lane == lane_num && v.s == s)
        {
        	this->vehicles.erase(v_id);
        }
        it++;
    }
    Vehicle ego = Vehicle(lane_num, s, this->lane_speeds[lane_num], 0);
    ego.configure(config_data);
    ego.state = "KL";
    this->vehicles.insert(std::pair<int,Vehicle>(ego_key,ego));
    
}
/*
void BehaviorPlanner::display(int timestep) {

    Vehicle ego = this->vehicles.find(this->ego_key)->second;
    int s = ego.s;
    string state = ego.state;

    this->camera_center = max(s, this->update_width/2);
    int s_min = max(this->camera_center - this->update_width/2, 0);
    int s_max = s_min + this->update_width;

    vector<vector<string> > road;

    for(int i = 0; i < this->update_width; i++)
    {
        vector<string> road_lane;
        for(int ln = 0; ln < this->num_lanes; ln++)
        {
            road_lane.push_back("     ");
        }
        road.push_back(road_lane);

    }

    map<int, Vehicle>::iterator it = this->vehicles.begin();
    while(it != this->vehicles.end())
    {

        int v_id = it->first;
        Vehicle v = it->second;

        if(s_min <= v.s && v.s < s_max)
        {
            string marker = "";
            if(v_id == this->ego_key)
            {
                marker = this->ego_rep;
            }
            else
            {
                
                stringstream oss;
                stringstream buffer;
                buffer << " ";
                oss << v_id;
                for(int buffer_i = oss.str().length(); buffer_i < 3; buffer_i++)
                {
                    buffer << "0";
                
                }
                buffer << oss.str() << " ";
                marker = buffer.str();
            }
            road[int(v.s - s_min)][int(v.lane)] = marker;
        }
        it++;
    }
    ostringstream oss;
    oss << "+Meters ======================+ step: " << timestep << endl;
    int i = s_min;
    for(int lj = 0; lj < road.size(); lj++)
    {
        if(i%20 ==0)
        {
            stringstream buffer;
            stringstream dis;
            dis << i;
            for(int buffer_i = dis.str().length(); buffer_i < 3; buffer_i++)
            {
                 buffer << "0";
            }
            
            oss << buffer.str() << dis.str() << " - ";
        }
        else
        {
            oss << "      ";
        }          
        i++;
        for(int li = 0; li < road[0].size(); li++)
        {
            oss << "|" << road[lj][li];
        }
        oss << "|";
        oss << "\n";
    }
    
    cout << oss.str();

}
*/
bool BehaviorPlanner::is_safe_to_move(int lane_num) {
//std::cout << "is_safe_to_move - lane_num:" << lane_num << " vehicles.size():" << vehicles.size() << std::endl;
//std::cout << "is_safe_to_move - [Ego]  ego_vehicle.s:" << ego_vehicle.s << " ego_vehicle.v:" << ego_vehicle.v << std::endl;

	bool is_safe = false;
	bool is_fore_safe = false;
	bool is_back_safe = false;
	int back_vehicle_s = 0;
	int back_vehicle_v = 0;
	int fore_vehicle_s = 0;
	int fore_vehicle_v = 0;
	
	map<int, Vehicle>::iterator it = this->vehicles.begin();
    while(it != this->vehicles.end())
    {
    	int v_id = it->first;
        Vehicle v = it->second;
//std::cout << "is_safe_to_move - v_id:" << v_id << " v.lane:" << v.lane << " v.s:" << v.s <<  " v.v:" << v.v << std::endl;

		if(v.lane == 0) {
			L_vehicle_num += 1;
		}
		if(v.lane == 2) {
			R_vehicle_num += 1;
		}
        if(v.lane == lane_num) {
//std::cout << "is_safe_to_move = v_id:" << v_id << " v.lane:" << v.lane << " v.s:" << v.s <<  " v.v:" << v.v << std::endl;

        	if (v.s >= ego_vehicle.s) { // fore vehicles
//std::cout << "LOOP FORE v_id:" << v_id << " v.lane:" << v.lane << " v.s:" << v.s <<  " v.v:" << v.v << std::endl;

        		if (fore_vehicle_v == 0) { // not yet initialized
//std::cout << "INIT FORE v_id:" << v_id << " v.lane:" << v.lane << " v.s:" << v.s <<  " v.v:" << v.v << std::endl;
        			fore_vehicle_s = v.s;
        			fore_vehicle_v = v.v;
        		} else{
        			if(fore_vehicle_s > v.s) { // check the nearest one
//std::cout << "UPDT FORE v_id:" << v_id << " v.lane:" << v.lane << " v.s:" << v.s <<  " v.v:" << v.v << std::endl;

        				fore_vehicle_s = v.s;
        				fore_vehicle_v = v.v;
        			}
        		}
        	} else { // back vehicles
//std::cout << "is_safe_to_move BACK v_id:" << v_id << " v.lane:" << v.lane << " v.s:" << v.s <<  " v.v:" << v.v << std::endl;

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
        it++;
    }
    //if(fore_vehicle_v == 0 || fore_vehicle_v > ego_vehicle.v || fore_vehicle_s > ego_lane_s) {
    //if(fore_vehicle_v == 0 || (fore_vehicle_s - ego_vehicle.s) > SAFE_DISTANCE || fore_vehicle_s > ego_lane_s) { EGO LANE?
	if (back_vehicle_s == 0 || (ego_vehicle.s - back_vehicle_s) > SAFE_DISTANCE) {
		is_back_safe = true;
//std::cout << "[Back is SAFE.]  back_vehicle_s:" << back_vehicle_s << " ego_vehicle.s:" << ego_vehicle.s << std::endl;

	} else {
std::cout << "[Back is not safe.] back_vehicle_s:" << back_vehicle_s << " ego_vehicle.s:" << ego_vehicle.s << std::endl;
	}

    if(fore_vehicle_v == 0 || (fore_vehicle_s - ego_vehicle.s) > SAFE_DISTANCE) {
		//if(fore_vehicle_v >= ego_vehicle.v) {
//std::cout << "[Fore is SAFE. Velocity OK]  fore_vehicle_v:" << fore_vehicle_v  << " ego_vehicle.v:" << ego_vehicle.v << std::endl;
			is_fore_safe = true;
		//} else {
//std::cout << "[Fore is SAFE. Distance OK]  fore_vehicle_s:" << fore_vehicle_s << " ego_vehicle.s:" << ego_vehicle.s << std::endl;
//}
	} else {
std::cout << "[Fore is not safe. Distance NG]  fore_vehicle_s:" << fore_vehicle_s  << " ego_vehicle.s:" << ego_vehicle.s << std::endl;
	}

	if (is_fore_safe && is_back_safe) {
		is_safe = true;
	}

	if(lane_num == 0) {
		farther_L_s = fore_vehicle_s;
		faster_L_v = fore_vehicle_v;
//std::cout << "is_safe_to_move SET L - farther_L_s:" << farther_L_s  << std::endl;

	}
	if(lane_num == 2) {
		farther_R_s = fore_vehicle_s;
		faster_R_v = fore_vehicle_v;
//std::cout << "is_safe_to_move SET R - farther_R_s:" << farther_R_s  << std::endl;
//std::cout << "is_safe_to_move SET R - farther_L_s:" << farther_L_s  << std::endl;
	}

    if (false) { //is_safe) {
	    if(lane_num == 0) {
	    	recommended_lane = lane_num;
			farther_lane_s = farther_L_s;
			faster_lane_speed = faster_L_v;
		}
		if(lane_num == 2) { 
    		if (farther_lane_s < farther_R_s) {
    			farther_lane_s = farther_R_s;
    			recommended_lane = lane_num;
    		}
    		if (fore_vehicle_s == 0) {// 0/LeftÇ…é‘Ç™Ç»Ç¢Ç∆Ç´Ç…êÑëEÇ≥ÇÍÇ»Ç¢ñ‚ëË
    			recommended_lane = lane_num;
    		}
    	} 
    }

//std::cout << "is_safe_to_move - recommended_lane:" << recommended_lane << " farther_lane_s:" << farther_lane_s << " faster_lane_speed:" << faster_lane_speed << std::endl;
//std::cout << "is_safe_to_move - [Right] farther_R_s:" << farther_R_s << " faster_R_v:" << faster_R_v << std::endl;
//std::cout << "is_safe_to_move - [Left]  farther_L_s:" << farther_L_s << " faster_L_v:" << faster_L_v << std::endl;
//std::cout << "is_safe_to_move - [Num of Cars]  L_vehicle_num:" << L_vehicle_num << " R_vehicle_num:" << R_vehicle_num << std::endl;

    return is_safe;
}
bool BehaviorPlanner::is_risky_to_move(double back_vehicle_v) {
	double elapsed_time = 0.1;
	return back_vehicle_v*elapsed_time >= ego_vehicle.s;
}

void BehaviorPlanner::add_ego(int lane_num, int s, double speed) {
	ego_vehicle = Vehicle(lane_num, s, speed, 0);
}
bool BehaviorPlanner::is_better_to_move() {
	faster_lane_speed = 0;
	farther_lane_s = 0;
	if(ego_vehicle.lane == 0) {
		return is_better_to_move(1);
	} else if (ego_vehicle.lane == 1) {
		return is_better_to_move(0) || is_better_to_move(1);
	} else if (ego_vehicle.lane == 2) {
		return is_better_to_move(1);
	}
	return false;
}
bool BehaviorPlanner::is_better_to_move(int lane_num) {
	bool is_move = false;
	map<int, Vehicle>::iterator it = this->vehicles.begin();
	int back_vehicle_s = 0;
	int back_vehicle_v = 0;
	int fore_vehicle_s = 0;
	int fore_vehicle_v = 0;
    while(it != this->vehicles.end())
    {
    	int v_id = it->first;
        Vehicle v = it->second;
        if(v.lane == lane_num) {
        	if (v.s >= ego_vehicle.s) {
        		if (fore_vehicle_v == 0) {
        			fore_vehicle_s = v.s;
        			fore_vehicle_v = v.v;
        		} else{
        			if(fore_vehicle_s > v.s) {
        				fore_vehicle_s = v.s;
        				fore_vehicle_v = v.v;
        			}
        		}
        	} else {
        		if (back_vehicle_v == 0) {
        			back_vehicle_s = v.s;
        			back_vehicle_v = v.v;
        		} else{
        			if(back_vehicle_s < v.s) {
        				back_vehicle_s = v.s;
        				back_vehicle_v = v.v;
        			}
        		}
        	}
        }
        it++;
    }
    
    if(back_vehicle_v != 0 || back_vehicle_v > ego_lane_speed) {
    	is_move = false;
    } else if(fore_vehicle_v == 0) {
    	is_move = true;
    	recommended_lane = lane_num;
    } else if (fore_vehicle_v > ego_lane_speed) { 
    	is_move = true;
    	if (faster_lane_speed < fore_vehicle_v) {
    		faster_lane_speed = fore_vehicle_v;
    		recommended_lane = lane_num;
    	} 
    }
    return is_move;
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
		map<int, Vehicle>::iterator it = this->vehicles.begin();
	    while(it != this->vehicles.end())
	    {
	    	int v_id = it->first;
	        Vehicle v = it->second;
std::cout << "recommend_lane - v_id:" << v_id << " v.lane:" << v.lane << " v.s:" << v.s <<  " v.v:" << v.v << std::endl;
			it++;
		}
	}
	return recommended_lane;
}