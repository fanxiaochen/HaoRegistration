#include "solver.h"

Solver::Solver(PointCloud *source, PointCloud *target)
    : source_(source),
      target_(target),
      rigid_alpha_(1),
      smooth_alpha_(1),
      fit_alpha_(1),
      conf_alpha_(1)
{

}

Solver::~Solver()
{

}

void Solver::initForTest()
{
    // rigid transformation
    source_->rigid_rot_[0] = 0;
    source_->rigid_trans_.setZero();

    //non-rigid transformation
    PointCloud::DeformationGraph *graph = source_->getDeformationGraph();
    ParameterMap *para_map = source_->getParameterMap();
    GraphMap *graph_map = source_->getGraphMap();
    for (PointCloud::DeformationGraph::NodeIt it(*graph); it != lemon::INVALID; ++ it) {
        Parameters &paras = (*para_map)[it];
      //  paras.affi_rot_ = Eigen::MatrixXd::Random(3, 3);
        paras.affi_rot_(0,0) = -0.4;
        paras.affi_rot_(1,1) = -0.4;
        paras.affi_rot_(2,2) = -0.4;
        
        paras.affi_trans_.setZero();
    }

}

void Solver::initParameters()
{
    // rigid transformation
    source_->rigid_rot_[0] = 0;
    source_->rigid_trans_.setZero();

    //non-rigid transformation
    PointCloud::DeformationGraph *graph = source_->getDeformationGraph();
    ParameterMap *para_map = source_->getParameterMap();
    GraphMap *graph_map = source_->getGraphMap();
    for (PointCloud::DeformationGraph::NodeIt it(*graph); it != lemon::INVALID; ++ it) {
        Parameters &paras = (*para_map)[it];
        paras.affi_rot_.setIdentity();
        paras.affi_trans_.setZero();
        paras.correspondence_[2] = 1;
    }
    pcl::PointCloud<Point>::Ptr target_knn(new pcl::PointCloud<Point>);
    target_knn->insert(target_knn->begin(), target_->begin(), target_->end());
    source_->getCorrespondenceByKnn(target_knn, target_); // knn for initial u, v
}

void Solver::buildProblem()
{
    PointCloud::DeformationGraph *graph = source_->getDeformationGraph();
    ParameterMap *para_map = source_->getParameterMap();
    GraphMap *graph_map = source_->getGraphMap();
    for (PointCloud::DeformationGraph::NodeIt it(*graph); it != lemon::INVALID; ++ it) {
        Parameters &paras = (*para_map)[it];

        //first energy term
        CostFunction *rigid_function = new ceres::AutoDiffCostFunction<RigidFunctor, 6, 9>(
            new RigidFunctor(rigid_alpha_));
        problem_.AddResidualBlock(rigid_function, NULL, paras.affi_rot_.data());

//         //second energy term
//         Eigen::Vector3d m_point = EIGEN_POINT_CAST(source_->at((*graph_map)[it]));
//         for (PointCloud::DeformationGraph::IncEdgeIt e(*graph, it); e != lemon::INVALID; ++ e) {
//             PointCloud::DeformationGraph::Node node = graph->target(e);
//             Eigen::Vector3d s_point = EIGEN_POINT_CAST(source_->at((*graph_map)[node]));
//             CostFunction *smooth_function = new ceres::AutoDiffCostFunction<SmoothFunctor, 3, 9, 3, 3>(
//                 new SmoothFunctor(smooth_alpha_, m_point, s_point));
//             problem_.AddResidualBlock(smooth_function, NULL, paras.affi_rot_.data(),
//                                       paras.affi_trans_.data(), (*para_map)[node].affi_trans_.data());
//         }

//         //third energy term
//         Eigen::Vector3d point = EIGEN_POINT_CAST(source_->at((*graph_map)[it]));
//         Eigen::Vector3d mass_center = source_->getMassCenter();
//         cv::Mat& depth_map = target_->getDepthMap();
//         CostFunction *fit_function = new ceres::NumericDiffCostFunction<FitFunctor, ceres::CENTRAL, 3, 2, 3, 3, 9, 3, 1>(
//             new FitFunctor(fit_alpha_, point, mass_center, &depth_map));
//         problem_.AddResidualBlock(fit_function, NULL, paras.correspondence_.data(),
//                                   source_->rigid_rot_.data(), source_->rigid_trans_.data(),
//                                   paras.affi_rot_.data(), paras.affi_trans_.data(),
//                                   paras.correspondence_.data()+2);

//         //forth energy term
//         CostFunction *conf_function = new ceres::AutoDiffCostFunction<ConfFunctor, 1, 1>(
//             new ConfFunctor(conf_alpha_));
//         problem_.AddResidualBlock(conf_function, NULL, (paras.correspondence_.data() + 2));
    }
}

// many parameters for controling the optimizer to get nice results
void Solver::setOptions()
{
    options_.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options_.minimizer_progress_to_stdout = true;
    options_.max_num_iterations = 1000;
}

void Solver::apply()
{
    ceres::Solve(options_, &problem_, &summary_);
    std::cout << summary_.FullReport() << std::endl;
}

void Solver::step()
{
    
}

void Solver::printParameters()
{
    PointCloud::DeformationGraph *graph = source_->getDeformationGraph();
    ParameterMap *para_map = source_->getParameterMap();
    GraphMap *graph_map = source_->getGraphMap();
    for (PointCloud::DeformationGraph::NodeIt it(*graph); it != lemon::INVALID; ++ it) {
        Parameters &paras = (*para_map)[it];
        paras.print();
    }
}
