// Pure-function checks for the WaitArrival budget (no ROS, plain host g++).
#include <lite3_bt/arrival_budget.hpp>

#include <cmath>
#include <cstdio>

static int failures = 0;

static void check(bool cond, const char* what)
{
  std::printf("%s  %s\n", cond ? "PASS" : "FAIL", what);
  if (!cond) ++failures;
}

int main()
{
  const double eps = 1e-9;
  check(std::abs(lite3_bt::arrival_budget_s(60.0, 0.0, 0.02) - 60.0) < eps,
        "no path keeps the tree timeout");
  check(std::abs(lite3_bt::arrival_budget_s(60.0, 1.2, 0.02) - 60.0) < eps,
        "short path keeps the tree timeout");
  check(std::abs(lite3_bt::arrival_budget_s(60.0, 7.5, 0.02) - 375.0) < eps,
        "ramp path scales to length/floor");
  check(std::abs(lite3_bt::arrival_budget_s(60.0, 7.5, 0.0) - 60.0) < eps,
        "zero floor disables scaling");
  check(std::abs(lite3_bt::arrival_budget_s(60.0, 7.5, -0.1) - 60.0) < eps,
        "negative floor disables scaling");
  check(std::abs(lite3_bt::arrival_budget_s(60.0, -1.0, 0.02) - 60.0) < eps,
        "negative length keeps the tree timeout");
  check(std::abs(lite3_bt::arrival_budget_s(60.0, NAN, 0.02) - 60.0) < eps,
        "NaN length keeps the tree timeout");
  check(std::abs(lite3_bt::arrival_budget_s(30.0, 7.5, 0.02) - 375.0) < eps,
        "scaling is independent of the base");
  return failures ? 1 : 0;
}
