#ifndef PID_H
#define PID_H

#include <vector>
#include <ctime>

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
  * returns time in seconds since last reset
  */
  double get_tuning_time( bool reset_clock );

  /*
  * tune PID parameters
  */
  void setup_tune();
  bool tune();

private:
  /* last cte */
  double prev_cte;

  /* sum of cte */
  double cte_sum;

  /* total error */
  double cte_square_sum;

  /* last update */
  std::chrono::high_resolution_clock::time_point time_last_update;

  /* tuning variables */
  unsigned int tune_current_param;
  int tune_iteration;
  int tune_direction;
  double tune_tolerance;
  double tune_err_best;
  std::vector<double> tune_dp;
  std::vector<double> tune_p;
  std::vector<double> tune_p_best;
  std::chrono::high_resolution_clock::time_point tuning_start_time;
};

#endif /* PID_H */
