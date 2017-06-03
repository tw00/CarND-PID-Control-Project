#include <math.h>       /* pow */
#include <iostream>
#include <chrono>
#include <numeric>
#include <ctime>
#include <cstdio>
#include <sys/time.h>

#include "PID.h"

using namespace std;
using namespace std::chrono;

/*
* PID class
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
    this->p_error = 0.0;
    this->i_error = 0.0;
    this->d_error = 0.0;
    this->prev_cte       = 0.0;
    this->cte_square_sum = 0.0;
    this->cte_sum        = 0.0;
    this->time_last_update  = high_resolution_clock::now();
    this->tuning_start_time = high_resolution_clock::now();
}

void PID::UpdateError(double cte) {

    high_resolution_clock::time_point now = high_resolution_clock::now();
    double sample_time = duration_cast<nanoseconds>( now - time_last_update ).count() / 1.0e9;
    time_last_update = now;
    // sample_time = 1.0; (use 1.0 for implementation without sample time)

    if( false ) cout << "dt = " << sample_time << " s" << endl;

    double diff_cte;
    diff_cte  = (cte - prev_cte ) / sample_time;

    cte_sum += cte * sample_time;
    cte_square_sum += pow( cte * sample_time, 2.0);

    p_error  = cte;
    i_error  = cte_sum;
    d_error  = diff_cte;
    prev_cte = cte;
}

double PID::TotalError() {

    double total_err = -( Kp*p_error + Ki*i_error + Kd*d_error );

    // could be improved by implementing anti windup
    if( total_err > 1.0 ) total_err = 1.0;
    if( total_err < -1.0 ) total_err = -1.0;

    return total_err;
}

void PID::setup_tune() {
    tune_current_param = 0;
    tune_iteration = 0;
    tune_direction = 1;
    tune_tolerance = 0.00001;
    tune_dp.assign(3, 1.0f);
    double param_p[]  = {Kp, Ki, Kd};
    double param_dp[] = {Kp/10.0f, Ki/10.0f, Kd/10.0f};
    tune_p.assign(param_p,param_p+3);
    tune_dp.assign(param_dp,param_dp+3);
    tune_err_best = 1e99;
}

double PID::get_tuning_time( bool reset_clock ) {

    high_resolution_clock::time_point now = high_resolution_clock::now();
    double tuning_time = duration_cast<nanoseconds>( now - tuning_start_time ).count() / 1.0e9;
    if( reset_clock ) tuning_start_time = now;
    return tuning_time;
}

bool PID::tune() {

    double tuning_time = this->get_tuning_time(true);
    // double err = (1.0 +  cte_square_sum) / pow( tuning_time, 3.0);
    double err = cte_square_sum / pow( tuning_time, 1.5);

    double sum_of_dp = std::accumulate(tune_dp.begin(), tune_dp.end(), 0.0);
    cout << "### ================================================" << endl;
    cout << "### TUNING got err " << err << " for run time of " << tuning_time << "s (current best: " << tune_err_best << ")" << endl;
    cout << "### TUNING params = (" << tune_p[0] << "," << tune_p[1] << "," << tune_p[2] << ")" << endl;
    cout << "### TUNING     dp = (" << tune_dp[0] << "," << tune_dp[1] << "," << tune_dp[2] << ")" << endl;

    if( sum_of_dp < tune_tolerance ) {
        cout << "### TUNING done, " << sum_of_dp << " below tolerance " << tune_tolerance << endl;
        return true;
    }
    cout << "### TUNING iteration [" <<  tune_iteration << "] param [" << tune_current_param << "] direction[" << tune_direction << "]" << endl;
    
    if( err < tune_err_best ) {
        cout << "### TUNING found improvement!" << endl;
        cout << "### TUNING ... for parameters Kp = " << Kp << ", Ki = " << Ki << ", Kd = " << Kd << " " << endl;
        tune_p_best = tune_p;
        tune_err_best = err; 
        tune_dp[tune_current_param] *= 1.1;
        tune_current_param += 1;
        tune_direction = +1;
    } else if( tune_direction == 0 ) {
        tune_dp[tune_current_param] *= 0.9;
        tune_current_param += 1;
        tune_direction = +1;
    } else {
        if( tune_direction == +1 ) {
            tune_direction = -1;
        } else if( tune_direction == -1 ) { 
            tune_direction = 0;
        } 
    }
    if( tune_current_param > 2 ) {
        tune_current_param = 0;
        tune_iteration += 1;
        tune_direction = +1;
    }

    if( tune_direction == +1 ) tune_p[tune_current_param] += tune_dp[tune_current_param];
    if( tune_direction == -1 ) tune_p[tune_current_param] -= tune_dp[tune_current_param];

    Kp = tune_p[0];
    Ki = tune_p[1];
    Kd = tune_p[2];
    this->Init(Kp, Ki, Kd);

    cout << "### TUNING next parameters Kp = " << Kp << ", Ki = " << Ki << ", Kd = " << Kd << " " << endl;
    cout << "### Simulating..." << endl;

    return false;
}
