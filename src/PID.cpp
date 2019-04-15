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
  this->di = 0;
  this->dp = 0;
  this->dd = 0;
  this->best_error = 10000000000;
  this->twiddle_mode = twiddle_mode_;
  this->it = 0;
  this->change_direction = false;
}

void PID::UpdateError(double cte)
{
  this->p_error = cte;
  this->i_error += cte;
  this->d_error = cte - this->p_error;
}

void PID::Twiddle(double tol, double cte)
{
  if (this->twiddle_mode == true)
  {
    double dsum = this->dp + this->dd + this->di;
    while (dsum > tol)
    {
      this->p_error += this->dp;
      this->i_error += this->di;
      this->d_error += this->dd;
      if (cte < this->best_error && this->change_direction == false)
      {
        this->best_error = cte;
        this->p_error *= 1.1;
        this->i_error *= 1.1;
        this->d_error *= 1.1;
      }
      else
      {
        this->change_direction == true;
        this->p_error -= 2 * this->dp;
        this->i_error -= 2 * this->di;
        this->d_error -= 2 * this->dd;
        if (this->change_direction == true)
        {
          if (cte < this->best_error)
          {
            this->best_error = cte;
            this->p_error *= 1.05;
            this->i_error *= 1.05;
            this->d_error *= 1.05;
          }
          else
          {
            this->p_error += this->dp;
            this->i_error += this->di;
            this->d_error += this->dd;
            this->p_error *= 0.95;
            this->i_error *= 0.95;
            this->d_error *= 0.95;
          }
          this->change_direction = false;
        }
      }
    }
  }
}

void PID::Reset()
{
  this->p_error = 0.0;
  this->d_error = 0.0;
  this->i_error = 0.0;
}

double PID::TotalError()
{
  double steering = this->p_error * this->Kp + this->i_error * this->Ki + this->d_error * this->Kd;
  return steering;
}