#include <aslam/backend/Optimizer2.hpp>
#include <aslam/backend/OptimizationProblem.hpp>
#include <aslam/backend/ErrorTerm.hpp>
#include <aslam/backend/Scalar.hpp>
#include <boost/make_shared.hpp>
#include <cmath>
#include <iostream>

/// Exponential residual whose undamped Newton proposal leaves its valid domain.
class DomainError final : public aslam::backend::ErrorTermFs<1> {
 public:
  /// Bind a scalar and choose recoverable, unrelated, or permanently invalid steps.
  DomainError(aslam::backend::Scalar* variable, int mode) : variable_(variable), mode_(mode) {
    setDesignVariables(variable_);
    setInvR(Eigen::Matrix<double, 1, 1>::Identity());
  }
  int rejected = 0;
  int regressions = 0;
 private:
  /// Reject only invalid proposals; the initial state at -2 is valid.
  double evaluateErrorImplementation() override {
    const double x = variable_->toScalar();
    if (x > 1.0 || (mode_ == 2 && x != -2.0)) {
      ++rejected;
      if (mode_ == 1) throw std::runtime_error("unrelated worker failure");
      throw aslam::SplineTimeOutOfBoundsException("test time support");
    }
    Eigen::Matrix<double, 1, 1> error;
    double residual = std::exp(x) - 2.0;
    // A deterministic numerical floor induces small rejected terminal steps,
    // after the solver has already found and retained a valid low-cost state.
    if (mode_ == 3 && std::abs(residual) < 1e-4) {
      residual = 0.01;
      ++regressions;
    }
    error << residual;
    setError(error);
    return evaluateChiSquaredError();
  }
  /// Differentiate the residual on its valid domain.
  void evaluateJacobiansImplementation(aslam::backend::JacobianContainer& jacobians) const override {
    Eigen::Matrix<double, 1, 1> jacobian;
    jacobian << std::exp(variable_->toScalar());
    jacobians.add(variable_, jacobian);
  }
  aslam::backend::Scalar* variable_;
  int mode_;
};

/// Check recovery, error propagation, and failure exhaustion with one or two workers.
static void check(int threads, int mode) {
  using namespace aslam::backend;
  const auto scalar = boost::make_shared<Scalar>(-2.0);
  scalar->setActive(true);
  const auto problem = boost::make_shared<OptimizationProblem>();
  problem->addDesignVariable(scalar);
  const auto first = boost::make_shared<DomainError>(scalar.get(), mode);
  problem->addErrorTerm(first);
  problem->addErrorTerm(boost::make_shared<DomainError>(scalar.get(), mode));
  Optimizer2Options options;
  options.nThreads = threads;
  options.maxIterations = mode == 2 ? 4 : 40;
  options.convergenceDeltaX = 1e-9;
  options.convergenceDeltaJ = 1e-12;
  options.trustRegionPolicy = boost::make_shared<LevenbergMarquardtTrustRegionPolicy>(1e-6);
  Optimizer2 optimizer(options);
  optimizer.setProblem(problem);
  bool propagated = false;
  try {
    const auto result = optimizer.optimize();
    if (mode == 0 && (result.linearSolverFailure || first->rejected == 0 ||
                      std::abs(scalar->toScalar() - std::log(2.0)) > 1e-6))
      throw std::runtime_error("LM did not recover from its invalid proposal");
    if (mode == 3 && (result.linearSolverFailure || first->regressions == 0 ||
                      std::abs(scalar->toScalar() - std::log(2.0)) > 1e-3))
      throw std::runtime_error("A numerically rejected terminal step discarded a valid solution");
    if (mode == 2 && (!result.linearSolverFailure || scalar->toScalar() != -2.0))
      throw std::runtime_error("Rejected-step exhaustion was reported as success");
  } catch (const std::runtime_error& error) {
    if (mode != 1 || std::string(error.what()) != "unrelated worker failure") throw;
    propagated = true;
  }
  if (mode == 1 && !propagated) throw std::runtime_error("Unrelated worker failure was hidden");
}

/// Exercise the actual threaded optimizer rather than an implementation mock.
int main() {
  try {
    for (int threads : {1, 2})
      for (int mode : {0, 1, 2, 3}) check(threads, mode);
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
