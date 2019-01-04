#ifndef LIDAR_H
#define LIDAR_H

#include <cstdint>
#include <iostream>

#define MAX_COLUMN_NUM 500
#define FREQUENCY 10
#define SAMPLES 25200                
#define PER_DEGREE 0.0142857142857143 
#define AMPLITUDE 650                 

class Lidar
{
public:
  Lidar();

  ~Lidar();

  unsigned int column;

  void result();

private:


  double coming_deg[25200]; 



  bool FLAG; 

  bool COINCIDENCE_FLAG;

  bool THEONE_COINCIDENCE_FLAG; 



  long int D_value; 

  double R_value;


};

#endif