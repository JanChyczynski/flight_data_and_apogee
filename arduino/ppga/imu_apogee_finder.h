#ifndef IMU_APOGEE_FINDER
#define IMU_APOGEE_FINDER

#include "imu_apogee_finder_params.h"

class Imu_apogee_finder
{
  int current_streak=0; //how many readings indicating an apogee have been obtained
  bool reached_apogee=0;
public:
  void insert_accelerations(imu_acc_t Axyz[])
  {
    imu_acc_t sum = 0.0;
    for (int axis=0; axis<3;axis++){
      if (axis != IMU_GRAVITY_AXIS){
        sum += Axyz[axis]* Axyz[axis];
      }
    }
    if (is_within_tolerance(sum) || 
      multiply_if_gr_neg(Axyz[IMU_GRAVITY_AXIS])<0){
      current_streak++;
    }else{
      current_streak = 0;
    }
    
    if (current_streak > IMU_STREAK_REQ){
      reached_apogee = 1;
    }else{
      reached_apogee = 0;
    }
  }
  bool get_reached_apogee(){
      return reached_apogee;
  }

private:
  bool is_within_tolerance(imu_acc_t value){
      return (value >= 1.0 - IMU_TOLERANCE && value <= 1.0 + IMU_TOLERANCE);
  }
  imu_acc_t multiply_if_gr_neg (imu_acc_t a){
      return IMU_IS_GRAVITY_NEGATIVE ? -a : a;
  }
};

#endif
