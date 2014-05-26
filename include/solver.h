#ifndef SOLVER_H
#define SOLVER_H

#include <unsupported/Eigen/NonLinearOptimization>
#include <unsupported/Eigen/NumericalDiff>

#include "point_cloud.h"

// Generic functor
template<typename _Scalar, int NX = Eigen::Dynamic, int NY = Eigen::Dynamic>
struct Functor {
    typedef _Scalar Scalar;
    enum {
        InputsAtCompileTime = NX,
        ValuesAtCompileTime = NY
    };
    typedef Eigen::Matrix<Scalar, InputsAtCompileTime, 1> InputType;
    typedef Eigen::Matrix<Scalar, ValuesAtCompileTime, 1> ValueType;
    typedef Eigen::Matrix<Scalar, ValuesAtCompileTime, InputsAtCompileTime> JacobianType;

    int m_inputs, m_values;

    Functor() : m_inputs(InputsAtCompileTime), m_values(ValuesAtCompileTime) {}
    Functor(int inputs, int values) : m_inputs(inputs), m_values(values) {}

    int inputs() const {
        return m_inputs;
    }
    int values() const {
        return m_values;
    }

};


struct EnergyFunction : Functor<double> {
    EnergyFunction(PointCloud *point_cloud)
        : _point_cloud(point_cloud) {
            init();
    }

    void init() {
        m_inputs = 16 * _point_cloud->getNodeNum() + 6; // the number of unknowns, a difference from original paper
        // m_values
    }

    int operator()(const Eigen::VectorXd &x, Eigen::VectorXd &fvec) const {
        // Implement y = 10*(x0+3)^2 + (x1-5)^2
        fvec(0) = 10.0 * pow(x(0) + 3.0, 2) +  pow(x(1) - 5.0, 2);
        fvec(1) = 0;

        return 0;
    }

    PointCloud *_point_cloud;
};


class Solver
{
public:
    Solver(EnergyFunction *energy_function);
    ~Solver();
    
    void apply();

private:
    EnergyFunction* energy_function_;
    Eigen::VectorXd x_;
};
#endif //SOLVER_H
