#ifndef PID_H
#define PID_H

#include <algorithm>
#include <iostream>

class PID {
 public:
  /**
   * Constructor
   */
  PID();

  /**
   * Destructor.
   */
  virtual ~PID();

  /**
   * Initialize PID.
   * @param (Kp_, Ki_, Kd_) The initial PID coefficients
   */
  void Init(double Kp_, double Ki_, double Kd_);

  void InitDP(std::vector<double> dp_);

  /**
   * Update the PID error variables given cross track error.
   * @param cte The current cross track error
   */
  void UpdateError(double cte);

  void UpdateParameter(int i, double ratio);

  void UpdateDP(int i, double ratio);

  /**
   * Calculate the total PID error.
   * @output The total PID error
   */
  double TotalError();
  
  double AverageError();

  double DPSum();
  
  void PrintPID();

 private:
  /**
   * PID Errors
   */
  double p_error;
  double i_error;
  double d_error;

  /**
   * PID Coefficients
   */ 
  double Kp;
  double Ki;
  double Kd;

  /**
   * Previous cross track error
   */ 
  double prev_cte;

  /**
   * Error counter
   */ 
  long error_counter;

  /**
   * Error sum
   */ 
  double error_sum;

  /**
   * Error max
   */
  double error_max;

  /**
   * Error min
   */
  double error_min;
  
  double error_best;

  std::vector<double> dp;
};

#endif  // PID_H