#ifndef PID_H
#define PID_H

#include <vector>
#include <string>
#include <iostream>
#include <cmath>

class PID
{

  enum class State
  {
    S1,
    S2,
    S3
  };

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
  void Init(double Kp_, double Ki_, double Kd_, bool twiddle_mode);

  /**
   * Update the PID error variables given cross track error.
   * @param cte The current cross track error
   */
  void UpdateError(double cte);

  /**
   * Twiddle function which is used to optimize the Kp/Ki/Kd
   * @param double tol
   */
  void Twiddle(double tol, double cte);

  /**
   * Calculate the total PID error.
   * @output The total PID error
   */
  double TotalError();

  /**
 * Resets the PID controller
 * 
 */
  void Reset();

  /**
  * Adds at the given index the given value
  * 
  */
  void AddAtIndex(int index, double value);

  /**
   * Print the vals
   */
  void PrintVals();

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
   * 
   */
  bool twiddle_mode;
  std::vector<double> dp;
  int it;
  double best_error;
  double all_error;
  int dp_index;
  State state;
};

#endif // PID_H