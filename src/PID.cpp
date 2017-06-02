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
//    this->sample_time = 1.0f;
    this->prev_cte    = 0.0;
    this->err_sum     = 0.0;
    this->time_last_update = duration_cast<milliseconds>( high_resolution_clock::now() );
    this->tuning_start_time = duration_cast<milliseconds>( high_resolution_clock::now() );
}

void PID::UpdateError(double cte) {

    milliseconds now = duration_cast<milliseconds>( high_resolution_clock::now() );
    double sample_time = ( duration_cast<std::chrono::duration<double>>(now - time_last_update).count() / 1000.0 );
    time_last_update = now;
//    sample_time = 1.0;

    double diff_cte;
    double steer;

//    cout << "dt = " << sample_time << endl;

    diff_cte  = (cte - prev_cte ) / sample_time;
    prev_cte  = cte;
    err_sum  += pow(cte, 2.0) * sample_time;

    i_error   = i_error + Ki * cte * sample_time;     
    p_error   = Kp * cte;
    d_error   = Kd * diff_cte;
}

double PID::TotalError() {

    double total_err;
    total_err = -( p_error + d_error + i_error );

    // TODO Anti Windup
    if( total_err > 1.0 ) total_err = 1.0;
    if( total_err < -1.0 ) total_err = -1.0;

    return total_err;
}

void PID::setup_tune() {
    tune_current_param = 0;
    tune_iteration = 0;
    tune_direction = 1;
    tune_tolerance = 0.2;
    tune_dp.assign(3, 1.0f);
    double param_p[]  = {Kp, Ki, Kd};
    double param_dp[] = {Kp/10.0f, Ki/10.0f, Kd/10.0f};
    tune_p.assign(param_p,param_p+3);
    tune_dp.assign(param_dp,param_dp+3);
    tune_err_best = 1e99;
}

bool PID::tune() {

    milliseconds now = duration_cast<milliseconds>( high_resolution_clock::now() );
    double tuning_time = ( duration_cast<std::chrono::duration<double>>(now - tuning_start_time).count() / 1000.0 );
    double err = err_sum / pow( tuning_time, 2.0);
    tuning_start_time = now;

    double sum_of_dp = std::accumulate(tune_dp.begin(), tune_dp.end(), 0);

    if( sum_of_dp < tune_tolerance ) {
        cout << "### TUNING done below tolerance " << tune_tolerance << endl;
        return true;
    }
    
    if( tune_current_param > 2 ) {
        tune_current_param = 0;
        tune_iteration += 1;
        tune_direction = +1;
    } else {
        if( tune_direction == +1 ) {
            tune_direction = -1;
        } else if( tune_direction == -1 ) { 
            tune_direction = 0;
        } else if( tune_direction == 0 ) {
            tune_current_param += 1;
            tune_direction = +1;
        }
    }
    cout << "### TUNING iteration [" <<  tune_iteration << "] param [" << tune_current_param << "] direction[" << tune_direction << "]" << endl;  
    cout << "### TUNING got err " << err << " for run time of " << tuning_time << "s" << endl;
    
    if( tune_direction == +1 ) tune_p[tune_current_param] += tune_dp[tune_current_param];
    if( tune_direction == -1 ) tune_p[tune_current_param] -= tune_dp[tune_current_param];

    if( err < tune_err_best ) {
        tune_p_best = tune_p;
        tune_err_best = err; 
        tune_dp[tune_current_param] *= 1.1;
        if( tune_direction == +1 ) { tune_current_param += 1; tune_direction = +1; }
        if( tune_direction == -1 ) { tune_current_param += 1; tune_direction = +1; }
    }

    if( tune_direction == 0 ) {
        tune_dp[tune_current_param] *= 0.9;
        tune_current_param += 1;
        tune_direction = +1;
    }

    Kp = tune_p[0];
    Ki = tune_p[1];
    Kd = tune_p[2];

    cout << "### TUNING next parameters Kp = " << Kp << ", Ki = " << Ki << ", Kd = " << Kd << " " << endl;

    return false;
}
