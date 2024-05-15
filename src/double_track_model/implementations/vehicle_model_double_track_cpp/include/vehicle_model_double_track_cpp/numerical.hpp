// Copyright 2023 Simon Sagmeister
#pragma once
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <vector>
namespace tam::helpers::numerical
{
template <typename T>
T integration_step_DoPri45(std::function<T(const double, const T)> f, double t, T x, double h)
{
  /*
  Do a single integration step with the Dormand-Prince integratation scheme.

  This integration scheme is usually used in the RK45 integrator.
  Code was taken from:
      https://stackoverflow.com/questions/54494770/how-to-set-fixed-step-size-with-scipy-integrate
  Additional references:
      https://en.wikipedia.org/wiki/List_of_Runge%E2%80%93Kutta_methods#Embedded_methods
      https://en.wikipedia.org/wiki/Dormand%E2%80%93Prince_method


  Args:
      f (std::function): callable function with the signature f(t,x)
      t (double): current time step in seconds
      x (T): the eigen array represeting the state vector
      h (double): step size in seconds for the integration step

  Returns:
      T: new state vector after integration step
  */

  // region taken from github
  // https://stackoverflow.com/questions/54494770/how-to-set-fixed-step-size-with-scipy-integrate
  T k1 = f(t, x);
  T k2 = f(t + 1.0 / 5 * h, x + h * (1.0 / 5 * k1));
  T k3 = f(t + 3.0 / 10 * h, x + h * (3.0 / 40 * k1 + 9.0 / 40 * k2));
  T k4 = f(t + 4.0 / 5 * h, x + h * (44.0 / 45 * k1 - 56.0 / 15 * k2 + 32.0 / 9 * k3));
  T k5 =
    f(t + 8.0 / 9 * h,
      x + h * (19372.0 / 6561 * k1 - 25360.0 / 2187 * k2 + 64448.0 / 6561 * k3 - 212.0 / 729 * k4));
  T k6 =
    f(t + h, x + h * (9017.0 / 3168 * k1 - 355.0 / 33 * k2 + 46732.0 / 5247 * k3 + 49.0 / 176 * k4 -
                      5103.0 / 18656 * k5));
  T v5 =
    35.0 / 384 * k1 + 500.0 / 1113 * k3 + 125.0 / 192 * k4 - 2187.0 / 6784 * k5 + 11.0 / 84 * k6;
  // T k7 = f(t + h, x + h * v5);

  T x_new = x + v5 * h;
  return x_new;
  // endregion
}
}
