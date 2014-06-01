#include "solver.h"

Solver::Solver(PointCloud *point_cloud)
    : point_cloud_(point_cloud)
{
    initCoeffs();
}

Solver::~Solver()
{

}

void Solver::initCoeffs()
{
    rigid_alpha_ = 1000;
    smooth_alpha_ = 100;
    fit_alpha_ = 0.1;
    conf_alpha_ = 100;
}

void Solver::buildProblem()
{
    PointCloud::DeformationGraph *graph = point_cloud_->getDeformationGraph();
    ParameterMap *para_map = point_cloud_->getParameterMap();
    GraphMap *graph_map = point_cloud_->getGraphMap();
    for (PointCloud::DeformationGraph::NodeIt it(*graph); it != lemon::INVALID; ++ it) {
        Parameters &paras = (*para_map)[it];
        
        //first energy term
        CostFunction *rigid_function = new ceres::AutoDiffCostFunction<RigidFunctor, 6, 9>(
            new RigidFunctor(rigid_alpha_));
        problem_.AddResidualBlock(rigid_function, NULL, paras.affi_rot_.data());
        
        //second energy term
        Eigen::Vector3d m_point = EIGEN_POINT_CAST(point_cloud_->at((*graph_map)[it]));
        for (PointCloud::DeformationGraph::IncEdgeIt e(*graph, it); e != lemon::INVALID; ++ e) {
            PointCloud::DeformationGraph::Node node = graph->target(e);
            Eigen::Vector3d s_point = EIGEN_POINT_CAST(point_cloud_->at((*graph_map)[node]));
            CostFunction *smooth_function = new ceres::AutoDiffCostFunction<SmoothFunctor, 3, 9, 3, 3>(
                new SmoothFunctor(smooth_alpha_, m_point, s_point));
            problem_.AddResidualBlock(smooth_function, NULL, paras.affi_rot_.data(), 
                                      paras.affi_trans_.data(), (*para_map)[node].affi_trans_.data());
        }
        
        //third energy term
        Eigen::Vector3d point = EIGEN_POINT_CAST(point_cloud_->at((*graph_map)[it]));
        Eigen::Vector3d mass_center = point_cloud_->getMassCenter();
        CostFunction *fit_function = new ceres::AutoDiffCostFunction<FitFunctor, 3, 2, 3, 3>(
            new FitFunctor(fit_alpha_, point, mass_center));
        // problem_.AddResidualBlock(fit_function, NULL, point_cloud_->rigid_rot_.data(), point_cloud_->rigid_trans_.data() );  lack of u,v parameters

        //forth energy term
        CostFunction *conf_function = new ceres::AutoDiffCostFunction<ConfFunctor, 1, 1>(
            new ConfFunctor(conf_alpha_));
        problem_.AddResidualBlock(conf_function, NULL, paras.correspondence_.data());

    }
}

// many parameters for controling the optimizer to get nice results
void Solver::setOptions()
{
    options_.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options_.minimizer_progress_to_stdout = true;
}

void Solver::apply()
{
    ceres::Solve(options_, &problem_, &summary_);
    std::cout << summary_.BriefReport() << std::endl;
}
