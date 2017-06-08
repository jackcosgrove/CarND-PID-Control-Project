#ifndef PID_H
#define PID_H

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;
  
  double max_total_error;
  double min_total_error;
  double total_error_range;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;
  
  /*
  * Twiddle parameters
  */
  double twiddle_error;
  double best_error;
  double twiddle_threshold;
  double dKp;
  double dKi;
  double dKd;
  int p_i;
  bool twiddling_up;
  bool twiddling_down;

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
  void Init(double Kp, double Ki, double Kd, double twiddle_threshold);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();
  
  /*
  * Scale the total error based on the max and min encountered
  */
  double ScaledTotalError(double min, double max);
  
  /*
  * Adjust the coefficient using the twiddle algorithm
  */
  void Twiddle();
};

#endif /* PID_H */
