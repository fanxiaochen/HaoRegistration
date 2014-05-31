
// #include "point_cloud.h"
// #include "visualizer.h"
//
// int main()
// {
//     PointCloud* point_cloud = new PointCloud();
//     point_cloud->load("/home/xiaochenfan/apple_1_1_8_depth.png");
//
//     Visualizer* visualizer = new Visualizer();
//     visualizer->init();
//     visualizer->put(point_cloud);
//     visualizer->visualize();
//
//     return 0;
// }
#include <iostream>

#include <eigen3/unsupported/Eigen/NonLinearOptimization>
#include <lemon/list_graph.h>


#include "ceres/ceres.h"
#include "glog/logging.h"
using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;

struct CostFunctor {
    template <typename T>
    bool operator()(const T *const x, T *residual) const {
        residual[0] = T(10.0) - x[0] - x[1];
        residual[1] = x[0] - x[1];
        return true;
    }
};

int main(int argc, char **argv)
{
// google::InitGoogleLogging(argv[0]);

    // The variable to solve for with its initial value.
    double initial_x = 5.0;
    double x[2];
    x[0] = initial_x;
    x[1] = 2.0;

    double y[2];
    y[0] = 5.0;
    y[1] = 4.0;

    // Build the problem.
    Problem problem;

    // Set up the only cost function (also known as residual). This uses
    // auto-differentiation to obtain the derivative (jacobian).
    CostFunction *cost_function =
        new AutoDiffCostFunction<CostFunctor, 2, 2>(new CostFunctor);
    problem.AddResidualBlock(cost_function, NULL, x);
    problem.AddResidualBlock(cost_function, NULL, y);

    // Run the solver!
    Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;
    options.max_num_iterations = 1;
    Solver::Summary summary;
    Solve(options, &problem, &summary);

    std::cout << summary.BriefReport() << "\n";
    std::cout << "x : "
              << " -> " << x[0] << " " << x[1] << "\n";
    std::cout << "y : "
              << " -> " << y[0] << " " << y[1] << "\n";
    return 0;
}
