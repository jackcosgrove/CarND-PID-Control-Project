#include "PID.h"

#include <iostream>
#include <limits>
#include <math.h>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd, double twiddle_threshold) {
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
  
  twiddle_error = 0.0;
  best_error = numeric_limits<double>::max();
  this->twiddle_threshold = twiddle_threshold;
  dKp = 0.5 * fabs(Kp);
  dKi = 0.5 * fabs(Ki);
  dKd = 0.5 * fabs(Kd);
  p_i = 2;
  twiddling_up = false;
  twiddling_down = false;
  
  min_total_error = -0.001;
  max_total_error = 0.001;
  total_error_range = max_total_error - min_total_error;
  
  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;
}

void PID::UpdateError(double cte) {
  p_error = cte;
  i_error += cte;
  d_error = cte - d_error;
  
  twiddle_error += cte;
}

double PID::TotalError() {
  double total_error = -Kp * p_error - Ki * i_error - Kd * d_error;
  if (total_error < 0) {
    if (total_error < min_total_error) {
      min_total_error = total_error;
    }
  } else if (total_error > max_total_error) {
    max_total_error = total_error;
  }
  total_error_range = max_total_error - min_total_error;
  return total_error;
}

double PID::ScaledTotalError(double min, double max) {
  double total_error = TotalError();
  double scaled_error_range = max - min;
  return (total_error - min_total_error) / total_error_range * scaled_error_range + min;
}

void PID::Twiddle() {
  if (dKp + dKi + dKd > twiddle_threshold) {
    if (!twiddling_up && !twiddling_down) {
      switch (p_i) {
        case 0:
          Kp += dKp;
          break;
        case 1:
          Ki += dKi;
          break;
        case 2:
          Kd += dKd;
          break;
      }
      twiddling_up = true;
    } else if (twiddling_up) {
      if (twiddle_error < best_error) {
        best_error = twiddle_error;
        switch (p_i) {
          case 0:
            dKp *= 1.1;
            break;
          case 1:
            dKi *= 1.1;
            break;
          case 2:
            dKd *= 1.1;
            break;
        }
      } else {
        twiddling_up = false;
        switch (p_i) {
          case 0:
            Kp -= 2.0 * dKp;
            if (Kp < 0.0) {
              Kp = 0.0;
            }
            break;
          case 1:
            Ki -= 2.0 * dKi;
            if (Ki < 0.0) {
              Ki = 0.0;
            }
            break;
          case 2:
            Kd -= 2.0 * dKd;
            if (Kd < 0.0) {
              Kd = 0.0;
            }
            break;
        }
        twiddling_down = true;
      }
    } else if (twiddling_down) {
      if (twiddle_error < best_error) {
        best_error = twiddle_error;
        switch (p_i) {
          case 0:
            dKp *= 1.2;
            break;
          case 1:
            dKi *= 1.2;
            break;
          case 2:
            dKd *= 1.2;
            break;
        }
      } else {
        twiddling_down = false;
        switch (p_i) {
          case 0:
            Kp += dKp;
            dKp *= 0.8;
            break;
          case 1:
            Ki += dKi;
            dKi *= 0.8;
            break;
          case 2:
            Kd += dKd;
            dKd *= 0.8;
            break;
        }
        
        p_i += 1;
        if (p_i > 2) {
          p_i = 0;
        }
      }
    }
    
    cout << "Twiddle P:" << Kp << dKp << endl;
    cout << "Twiddle I:" << Ki << dKi << endl;
    cout << "Twiddle D:" << Kd << dKd << endl;
    
    twiddle_error = 0.0;
  }
}

