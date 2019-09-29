#include "PID.h"

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */
  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;

  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;
  error_best = std::numeric_limits<double>::max();

  prev_cte = 0.0;

  error_counter = 0;
  error_sum = 0.0;
  error_max = std::numeric_limits<double>::min();
  error_min = std::numeric_limits<double>::max();
}

void PID::InitDP(std::vector<double> dp_){
  dp = dp_;
}

void PID::UpdateParameter(int i, double ratio){
  switch(i){
    case 0:
      Kp += ratio * dp[i];
      break;
    case 1:
      Ki += ratio * dp[i];
      break;
    case 2:
      Kd += ratio * dp[i];
      break;
    default:
      break;
  }
}

void PID::UpdateDP(int i, double ratio){
  dp[i] *= ratio;
}


void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */

  // proportional error
  p_error = cte;

  // integral error
  i_error += cte;

  // differential error
  d_error = cte - prev_cte;

  // update previous cte
  prev_cte = cte;

  error_sum += cte;
  error_counter++;

  if(cte > error_max){
    error_max = cte;
  }

  if(cte < error_min){
    error_min = cte;
  }
}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */
  return p_error * Kp + i_error * Ki + d_error * Kd;  // TODO: Add your total error calc here!
}

double PID::AverageError(){
  return error_sum / error_counter;
}

double PID::DPSum(){
  return std::accumulate(dp.begin(), dp.end(), 0.0);
}

void PID::PrintPID(){
  std::cout << "PID: " << std::to_string(Kp) << ", " << std::to_string(Ki) << ", " << std::to_string(Kd) << std::endl;
  std::cout << "dp: " << std::to_string(dp[0]) << ", " << std::to_string(dp[1]) << ", " << std::to_string(dp[2]) << std::endl;
}