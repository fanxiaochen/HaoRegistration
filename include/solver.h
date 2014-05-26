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
        m_inputs = 15 * _point_cloud->getNodeNum() + 6; // the number of unknowns
        // m_values  the number of equations
    }

    int operator()(const Eigen::VectorXd &x, Eigen::VectorXd &fvec) const {
        // Implement the equations
        size_t findex = 0;

        for (size_t i = 0, i_end = _point_cloud->getNodeNum(); i < i_end; i ++) {
            fvec(findex ++) = pow(x(15*i) * x(15*i+3) + x(15*i+1) * x(15*i+4) + x(15*i+2) * x(15*i+5), 2);
            fvec(findex ++) = pow(x(15*i) * x(15*i+6) + x(15*i+1) * x(15*i+7) + x(15*i+2) * x(15*i+8), 2);
            fvec(findex ++) = pow(x(15*i+3) * x(15*i+6) + x(15*i+4) * x(15*i+7) + x(15*i+5) * x(15*i+8), 2);
            fvec(findex ++) = pow(1 - x(15*i) * x(15*i) + x(15*i+1) * x(15*i+1) + x(15*i+2) * x(15*i+2), 2);
            fvec(findex ++) = pow(1 - x(15*i+3) * x(15*i+3) + x(15*i+4) * x(15*i+4) + x(15*i+5) * x(15*i+5), 2);
            fvec(findex ++) = pow(1 - x(15*i+6) * x(15*i+6) + x(15*i+7) * x(15*i+7) + x(15*i+8) * x(15*i+8), 2);
            
        }
                    

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
    EnergyFunction *energy_function_;
    Eigen::VectorXd x_;
};
#endif //SOLVER_H
