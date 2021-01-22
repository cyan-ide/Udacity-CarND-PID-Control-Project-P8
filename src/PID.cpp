#include "PID.h"
#include <cmath>
#include <iostream>

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

static const double TOLERANCE = 0.2;

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */
    pid_coefficients = {Kp_, Kd_, Ki_};
    
    //init twiddle params
    //dp = {1, 1, 1}; //coefficient delta (ie. the amount coefficients get updated per twiddle step
    dp = {Kp_/ 10, Kd_/ 10, Ki_/ 10}; //coefficient delta (ie. the amount coefficients get updated per twiddle step
    //zero out variables for calculation of avg error (ie. running twiddle only every "x" steps on avg error, rather than every single robot movement)
    cte_sum = 0.0;
    cte_sum_counter = 0;
    best_avg_cte = 1.0;
}

void PID::UpdateError(double cte) {
    d_error = cte - p_error; //p_error == previous value of cte
    p_error = cte;
    i_error += cte;
}

//run twiddle to update coeffictients (one coefficient at at time, the status which coeffcient is modified is stored in class variable)
void PID::UpdateCoefficients(double cte) {
    
    double coefficient_detla_sum = dp[0] + dp[1] + dp[2]; //sum of coefficient deltas = used for comparison to some tolarance to decide if twiddle did sufficient updates or not
    
    if (coefficient_detla_sum < TOLERANCE) { //if tolerance for coefficients delta met; stop updating (ie. this happens when updates have become so small that we dont care anymore)
        return;
    }
    
    if ((cte_sum_counter!= 0) && (cte_sum_counter % ERROR_SUM_SAMPLE_SIZE == 0)) { //if reached enough robot movements, check avg error value and run twiddle
        double avg_cte = cte_sum / cte_sum_counter; //caluclate avg error from last "ERROR_SUM_SAMPLE_SIZE" steps
        cte_sum = 0; //zero error sum
        cte_sum_counter = 0; //reset sum counter
        
        //do the twindle algorithm to update coefficients based on avg error value
        //alg runs in 3 steps:
        // - (0) increment coefficient;
        // - (1) if error decreased increment coefficient delta and go step (0); OR if error increased deincrement coefficient and go to step (2)
        // - (2) if error decreased increment coefficient delta and go to step (0); if error increased increment coefficient. increment delta by smaller value and go to step (0)
        if (twiddle_update_step == 0) { //first alg step by default it to increase coefficient
            pid_coefficients[coefficient_num] += dp[coefficient_num]; //increment coefficient
            twiddle_update_step++; //set to next alg step and return to <run 100 loops car movement> to see how the error changes
        } else if (twiddle_update_step == 1) { //alg update after coefficient increase in previous step
            if (avg_cte < best_avg_cte) { //if prior change was good (ie. avg error decreased)
                best_avg_cte = avg_cte; //update best error
                dp[coefficient_num] *= 1.1; //increment coefficient delta
                
                
                //---
                twiddle_update_step = 0; //since the change was good, restart twiddle state and return to <run 100 loops car movement> to update the error
                //move to next coefficient
                if (coefficient_num<2) {
                    coefficient_num++;
                } else {
                    coefficient_num=0;
                }
                //---
            } else { //if prior change was bad (ie. avg error increased)
                pid_coefficients[coefficient_num] -= 2*dp[coefficient_num]; //rollback previous increment and deincrement coefficient
                //--
                twiddle_update_step++; //set to next alg step and return to <run 100 loops car movement> to update the error
                //--
            }
        } else if (twiddle_update_step == 2) { //alg update after coefficient decrease in previous step
            if (avg_cte < best_avg_cte) { //if prior change was good (ie. avg error decreased)
                best_avg_cte = avg_cte; //update best error
                dp[coefficient_num] *= 1.1; //increment coefficient delta
            } else { //if prior change was bad (ie. avg error increased)
                pid_coefficients[coefficient_num] += dp[coefficient_num]; //rollback the previous coefficient deincrement
                dp[coefficient_num] *= 0.9; //increment coefficient delta by smaller value than before
            }
            
            //---
            twiddle_update_step = 0; //restart twiddle state and return to <run 100 loops car movement> to update the error
            //move to next coefficient
            if (coefficient_num<2) {
                coefficient_num++;
            } else {
                coefficient_num=0;
            }
            //---
        }
    } else //if not enough data collected keep recording error
    {
        //cte_sum += cte; //add error
        //cte_sum += cte*cte; //add error
        cte_sum += fabs(cte); //add error
        cte_sum_counter += 1; //increase sum counter
    }
}

double PID::TotalError() {
    double steer = -pid_coefficients[0] * p_error - pid_coefficients[1] * d_error - pid_coefficients[2] * i_error;
    
    UpdateCoefficients(p_error); //run twiddle to update coefficients
    return steer;
}
