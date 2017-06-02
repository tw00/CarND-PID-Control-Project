#ifndef PID_H
#define PID_H

#include <vector>

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  /*
  * tune PID parameters
  */
  void setup_tune();
  bool tune();

private:
  /* last cte */
  double prev_cte;

  /* total error */
  double err_sum;

  /* last update */
  std::chrono::milliseconds time_last_update;

  /* tuning variables */
  unsigned int tune_current_param;
  int tune_iteration;
  int tune_direction;
  double tune_tolerance;
  double tune_err_best;
  std::vector<double> tune_dp;
  std::vector<double> tune_p;
  std::vector<double> tune_p_best;
  std::chrono::milliseconds tuning_start_time;
};

#endif /* PID_H */
