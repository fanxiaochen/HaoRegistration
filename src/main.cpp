
#include "point_cloud.h"
#include "visualizer.h"
#include "solver.h"
#include "event_handler.h"
#include "scene_viewer.h"

int main()
{
    PointCloud* source = new PointCloud();
    PointCloud* target = new PointCloud();
    source->load("/home/xiaochenfan/images/0.png", false);
    target->load("/home/xiaochenfan/images/5.png", false);
    source->setColor(121, 5, 237);
    target->setColor(121, 237, 5);
    source->setNodeNum(200);
    source->binding();
    source->smoothDependency();
    
    Solver* solver = new Solver(source, target);
    solver->initParameters();
 //   solver->initForTest();
    solver->buildProblem();
    solver->setOptions();
//    solver->apply();    
//    solver->printParameters();
 //   source->update();
    
    Visualizer* visualizer = new Visualizer();
    visualizer->closeLight();
    visualizer->addPointCloud(source);
    visualizer->addPointCloud(target);
  //  visualizer->addGraph(source);
    
    EventHandler* event_handler = new EventHandler(solver, visualizer);
    
    SceneViewer* scene_viewer = new SceneViewer(visualizer, event_handler);
    scene_viewer->init();
    scene_viewer->start();

    return 0;
}
// #include <iostream>
// 
// #include <eigen3/unsupported/Eigen/NonLinearOptimization>
// #include <lemon/list_graph.h>
// 
// 
// #include "ceres/ceres.h"
// #include "glog/logging.h"
// using ceres::AutoDiffCostFunction;
// using ceres::CostFunction;
// using ceres::Problem;
// using ceres::Solver;
// using ceres::Solve;
// 
// struct CostFunctor {
//     template <typename T>
//     bool operator()(T const* const* x, T *residual) const {
//         T const* a = x[0];
//         T const* b = x[1];
//         residual[0] = 4.0*(T(10.0) - a[0] - b[0]);
//         residual[1] = 2.0*b[0] - a[1] * b[1];
//         return true;
//     }
// };
// 
// // class Test
// // {
// // public:
// //     Test(double* x){x_ = x;}
// //     double compute(){return 2*x_[0] - x_[1];}
// // private:
// //     double* x_;
// // };
// // 
// // struct TestFunctor {
// //     TestFunctor(Test* t){t_ = t;}
// //     template <typename T>
// //     bool operator()(const T *const x, T *residual) const {
// //         residual[0] = t_->compute() * x[0] - 2 * x[1];
// //         return true;
// //     }
// // private:
// //     Test* t_;
// // };
// 
// 
// 
// int main(int argc, char **argv)
// {
// // google::InitGoogleLogging(argv[0]);
// 
//     // The variable to solve for with its initial value.
//     
//     double x[2] = {10, 10};
//     double y[2] = {5, 5};
//     std::vector<double*> tmp;
//     tmp.push_back(x);
//     tmp.push_back(y);
//   //  Test t(x);
// 
//     // Build the problem.
//     Problem problem;
// 
//     // Set up the only cost function (also known as residual). This uses
//     // auto-differentiation to obtain the derivative (jacobian).
//     ceres::DynamicNumericDiffCostFunction<CostFunctor> *cost_function =
//         new ceres::DynamicNumericDiffCostFunction<CostFunctor>(new CostFunctor);
//     cost_function->AddParameterBlock(2);
//     cost_function->AddParameterBlock(2);
//     cost_function->SetNumResiduals(2);
//     
// //     CostFunction *test_function =
// //         new ceres::NumericDiffCostFunction<TestFunctor, ceres::CENTRAL, 1, 2>(new TestFunctor(&t));    
//     problem.AddResidualBlock(cost_function, NULL, tmp);
//  //   problem.AddResidualBlock(test_function, NULL, x);
// 
//     // Run the solver!
//     Solver::Options options;
//     options.linear_solver_type = ceres::DENSE_QR;
//     options.minimizer_progress_to_stdout = true;
//     options.max_num_iterations = 100;
//     Solver::Summary summary;
//     Solve(options, &problem, &summary);
// 
//     std::cout << summary.BriefReport() << "\n";
//     std::cout << "x : "
//               << " -> " << x[0] << " " << x[1] << "\n";
//     std::cout << "y : "
//               << " -> " << y[0] << " " << y[1] << "\n";
//   
//     return 0;
// }
