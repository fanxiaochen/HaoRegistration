#ifndef SOLVER_H
#define SOLVER_H

#include <unsupported/Eigen/NonLinearOptimization>
#include <unsupported/Eigen/NumericalDiff>
#include <boost/concept_check.hpp>

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
    
    inline double multiply(const double& x1, const double& y1, const double& z1, 
                           const double& x2, const double& y2, const double& z2) const {
    
        return x1*x2 + y1*y2 + z1*z2;
    }
    
    inline double multiply(const double& x, const double& y, const double& z) const {
    
        return x*x + y*y + z*z;
    }
    
    inline double multiply(const double& x, const double& y) const {
    
        return x*x + y*y;
    }
    
//     inline double multiply(double& a11, double& a12, double& a13, double& a21, double& a22, double& a23,
//         double& a31, double& a32, double& a33, double& x, double& y, double& z)
//     {
//         return a11;
//     }

    int operator()(const Eigen::VectorXd &x, Eigen::VectorXd &fvec) const {
        // Implement the equations
        size_t findex = 0;
        size_t node_num = _point_cloud->getNodeNum();
        PointCloud::DeformationGraph* graph = _point_cloud->getDeformationGraph();
        GraphMap* graph_map = _point_cloud->getGraphMap();
        ParameterMap* parameter_map = _point_cloud->getParameterMap();
        
        for (PointCloud::DeformationGraph::NodeIt it(*graph); it != lemon::INVALID; ++ it) {
            size_t node_id = graph->id(it);
            
            // first energy term
            fvec(findex ++) = pow(multiply(x(15*node_id),x(15*node_id+1),x(15*node_id+2),
              x(15*node_id+3),x(15*node_id+4),x(15*node_id+5)  
            ), 2);
            fvec(findex ++) = pow(multiply(x(15*node_id),x(15*node_id+1),x(15*node_id+2),
                x(15*node_id+6),x(15*node_id+7),x(15*node_id+8)
            ), 2);
            fvec(findex ++) = pow(multiply(x(15*node_id+3),x(15*node_id+4),x(15*node_id+5),
                x(15*node_id+6),x(15*node_id+7),x(15*node_id+8)
            ), 2);
            
            fvec(findex ++) = pow(1-multiply(x(15*node_id),x(15*node_id+1),x(15*node_id+2)), 2);
            fvec(findex ++) = pow(1-multiply(x(15*node_id+3),x(15*node_id+4),x(15*node_id+5)), 2);
            fvec(findex ++) = pow(1-multiply(x(15*node_id+6),x(15*node_id+7),x(15*node_id+8)), 2);
            
            //second energy term
            int i = (*graph_map)[it];
            Point i_point = _point_cloud->at(i);
            for (PointCloud::DeformationGraph::IncEdgeIt e(*graph, it); e != lemon::INVALID; ++ e){
                PointCloud::DeformationGraph::Node node = graph->target(e);
                size_t target_id = graph->id(node);
                int j = (*graph_map)[node];
                Point j_point = _point_cloud->at(j);
                
                double x_smooth = multiply(x(15*node_id),x(15*node_id+3),x(15*node_id+6),
                    (j_point.x-i_point.x),(j_point.y-i_point.y),(j_point.z-i_point.z)
                ) + i_point.x + x(15*node_id+9) - (j_point.x + x(15*target_id+9));
                
                double y_smooth = multiply(x(15*node_id+1),x(15*node_id+4),x(15*node_id+7),
                    (j_point.x-i_point.x),(j_point.y-i_point.y),(j_point.z-i_point.z)
                ) + i_point.y + x(15*node_id+10) - (j_point.y + x(15*target_id+10));
                  
                double z_smooth = multiply(x(15*node_id+2),x(15*node_id+5),x(15*node_id+8),
                    (j_point.x-i_point.x),(j_point.y-i_point.y),(j_point.z-i_point.z)
                ) + i_point.z + x(15*node_id+11) - (j_point.z + x(15*target_id+11));
                
                fvec(findex ++) = pow(multiply(x_smooth, y_smooth, z_smooth), 2);
            }
            
            //third energy term, need range image supports
            double z = 0; // stand for the z from range image 
            
            //forth energy term
            fvec(findex ++) = pow(1-pow(x(15*node_id+14),2),2);
            
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
