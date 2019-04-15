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
  this->dp = {1,1,1};
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
    double dsum = this->dp[0] + this->dp[1] + this->dp[2];
    if (dsum <= tol)
    {
      this->twiddle_mode = false;
      std::cout << "Kp: " << this->Kp << ", Ki: " << this->Ki << ", Kd:" << this->Kd << std::endl;
      return;
    }

    all_error += pow(cte, 2.0);

    switch (state)
    {
    case State::S1:
      // use dp_index to add the values to every controller parameter
      switch (dp_index)
      {
      case 0:
        Kp += this->dp[0];
        break;
      case 1:
        Ki += this->dp[1];
        break;
      case 2:
        Kd += this->dp[2];
        break;
      }
      state = State::S2;
      break;

    case State::S2:
      if (all_error < this->best_error)
      {
        this->best_error = all_error;
        this->dp[dp_index] *= 1.1;
        state = State::S1;
      }
      else
      {
        switch (dp_index)
        {
        case 0:
          Kp -= 2 * this->dp[0];
          break;
        case 1:
          Ki -= 2 * this->dp[1];
          break;
        case 2:
          Kd -= 2 * this->dp[2];
          break;
        }
        state = State::S3;
      }
      break;

    case State::S3:
      if (all_error < this->best_error)
      {
        this->best_error = all_error;
        this->dp[dp_index] *= 1.05;
      }
      else
      {
        switch (dp_index)
        {
        case 0:
          Kp += this->dp[0];
          break;
        case 1:
          Ki += this->dp[1];
          break;
        case 2:
          Kd += this->dp[2];
          break;
        }
        this->dp[dp_index] *= 0.95;
      }
      state = State::S1;
      break;
    }
  }
  all_error = 0;
  if (dp_index != 2)
  {
    dp_index = dp_index + 1;
  }
  else
  {
    dp_index = 0;
  }
}

void PID::Reset()
{
  this->p_error = 0.0;
  this->d_error = 0.0;
  this->i_error = 0.0;
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