#include "PID.h"

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_, bool twiddle_mode_)
{
  this->Kp = Kp_;
  this->Ki = Ki_;
  this->Kd = Kd_;
  this->p_error = 0;
  this->i_error = 0;
  this->d_error = 0;
  //this->dp = {0.1, 0.0001, 1.0};
  this->dp = {0.05, 0.0001, 1.0};
  this->dp_index = 0;
  this->best_error = 10000000000;
  this->all_error = 0;
  this->state = State::S1;
  this->twiddle_mode = twiddle_mode_;
  this->it = 0;
}

void PID::UpdateError(double cte)
{
  d_error = cte - this->p_error;
  p_error = cte;
  i_error += cte;
}

void PID::Twiddle(double tol, double cte)
{
  if (this->twiddle_mode == true)
  {
    all_error += pow(cte, 2);
    std::cout << "All Error:" << all_error << std::endl;
    switch (state)
    {
    case State::S1:
      // use dp_index to add the values to every controller parameter
      AddAtIndex(dp_index, dp[dp_index]);
      state = State::S2;
      break;

    case State::S2:
      if (all_error < best_error)
      {
        std::cout << "Improvement:" << std::endl;
        best_error = all_error;
        dp[dp_index] *= 1.1;
        dp_index += 1;
        state = State::S1;
      }
      else
      {
        AddAtIndex(dp_index, -2 * dp[dp_index]);
        state = State::S3;
      }
      break;

    case State::S3:
      if (all_error < best_error)
      {
        best_error = all_error;
        dp[dp_index] *= 1.1;
      }
      else
      {
        AddAtIndex(dp_index, dp[dp_index]);
        dp[dp_index] *= 0.90;
      }
      dp_index += 1;
      state = State::S1;
      break;
    }
  }
  all_error = 0;
  dp_index %= 3;
}

void PID::Reset()
{
  this->p_error = 0.0;
  this->d_error = 0.0;
  this->i_error = 0.0;
}

void PID::AddAtIndex(int index, double value)
{
  switch (index)
  {
  case 0:
    Kp += value;
    break;
  case 1:
    Ki += value;
    break;
  case 2:
    Kd += value;
    break;
  }
}

void PID::PrintVals()
{
  std::cout << "Kp: " << this->Kp << ", Ki: " << this->Ki << ", Kd:" << this->Kd << std::endl;
}

double PID::TotalError()
{
  double steering = -1 * (this->p_error * this->Kp + this->i_error * this->Ki + this->d_error * this->Kd);
  return steering;
}