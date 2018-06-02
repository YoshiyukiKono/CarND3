#ifndef VEHICLE_H
#define VEHICLE_H

class Vehicle {
public:

  int lane;

  int s;

  float v;

  float a;

  float target_speed;

  /**
  * Constructor
  */
  Vehicle();
  Vehicle(int lane, float s, float v, float a);
  /**
  * Destructor
  */
  virtual ~Vehicle();

};

#endif