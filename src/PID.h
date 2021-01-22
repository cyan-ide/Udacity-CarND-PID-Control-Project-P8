#ifndef PID_H
#define PID_H

#include <vector>

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

  /**
   * Update the PID error variables given cross track error.
   * @param cte The current cross track error
   */
  void UpdateError(double cte);
        
    /**
     * Update the PID coefficients using Twiddle algorithm
     * @param cte The current cross track error
     */
  void UpdateCoefficients(double cte);

  /**
   * Calculate the total PID error.
   * @output The total PID error
   */
  double TotalError();
    
    
    // -------
    /**
     * Set coefficient value. The coefficient to set is defined by "coefficient_num".
     * (added this function  to keep the PID class consistant with Udacity header (instead of defining coefficient table)
     * @param cte The current cross track error
     */
//  void PID::SetCoefficient(double value, int coefficient_num)

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
  std::vector<double> pid_coefficients; //Kp , Ki, Kd
    
 /** Twiddle parameters */
  std::vector<double> dp;
  //total cte error calculation variables (ie. twiddle checks avg error every 'x' steps, rather than every single car/robot movement)
  double cte_sum; //sum of errors
  int cte_sum_counter; //count of errors added
  static const int ERROR_SUM_SAMPLE_SIZE = 10; //amount of steps for avg error calculation
  //variables related to subsequent twiddle iterations (ie. changing subsequent coefficients and making update decision based on prior iteration)
  int coefficient_num;
  double best_avg_cte;
  int twiddle_update_step; //twiddle algorithm steps to update coeffcient are split with car/robot movements to collect error and react on it
  
};

#endif  // PID_H
