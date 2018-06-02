#include "vehicle.h"

/**
 * Initializes Vehicle
 */
Vehicle::Vehicle() {}
Vehicle::Vehicle(int lane, float s, float v, float a) {

    this->lane = lane;
    this->s = s;
    this->v = v;
    this->a = a;

}

Vehicle::~Vehicle() {}
