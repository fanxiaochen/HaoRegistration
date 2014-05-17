#include "solver.h"

Solver::Solver(EnergyFunction *energy_function)
    : energy_function_(energy_function),
      x_(energy_function->inputs())
{

}

Solver::~Solver()
{

}

void Solver::apply()
{
    Eigen::NumericalDiff<EnergyFunction> numDiff(*energy_function_);
    Eigen::LevenbergMarquardt<Eigen::NumericalDiff<EnergyFunction>, double> lm(numDiff);
    lm.parameters.maxfev = 2000;
    lm.parameters.xtol = 1.0e-10;
    std::cout << lm.parameters.maxfev << std::endl;

    int ret = lm.minimize(x_);
    std::cout << lm.iter << std::endl;
    std::cout << ret << std::endl;
}
