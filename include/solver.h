#ifndef SOLVER_H
#define SOLVER_H

#include <ceres/ceres.h>
#include <glog/logging.h>

#include "point_cloud.h"

#define INNER_PRODUCT(x1, y1, z1, x2, y2, z2) (((x1)*(x2))+((y1)*(y2))+((z1)*(z2)))
#define SQUARE_NORM(x, y, z) (((x)*(x))+((y)*(y))+((z)*(z)))


// default column-major in eigen
struct RigidFunctor {
    RigidFunctor(double coeff)
        : _coeff(coeff) {

    }
    template <typename T>
    bool operator()(const T *const affi_m, T *residual) const {
        residual[0] = INNER_PRODUCT(affi_m[0], affi_m[1], affi_m[2], affi_m[3], affi_m[4], affi_m[5]);
        residual[1] = INNER_PRODUCT(affi_m[0], affi_m[1], affi_m[2], affi_m[6], affi_m[7], affi_m[8]);
        residual[2] = INNER_PRODUCT(affi_m[3], affi_m[4], affi_m[5], affi_m[6], affi_m[7], affi_m[8]);
        residual[3] = T(1) - SQUARE_NORM(affi_m[0], affi_m[1], affi_m[2]);
        residual[4] = T(1) - SQUARE_NORM(affi_m[3], affi_m[4], affi_m[5]);
        residual[5] = T(1) - SQUARE_NORM(affi_m[6], affi_m[7], affi_m[8]);

        for (size_t i = 0; i < 6; i ++) {
            residual[i] = _coeff * residual[i];
        }

        return true;
    }

private:
    double _coeff;
};

struct SmoothFunctor {
    SmoothFunctor(double coeff, Eigen::Vector3d master, Eigen::Vector3d slave)
        : _coeff(coeff) , _master(master), _slave(slave) {
    }

    template <typename T>
    bool operator()(const T *const affi_m, const T *const trans_m, const T *const trans_s, T *residual) const {
        Eigen::Vector3d delta_2 = _slave - _master;

        residual[0] = INNER_PRODUCT((affi_m[0] - T(1)), affi_m[3], affi_m[6], 
                                    delta_2(0), delta_2(1), delta_2(2)) + trans_m[0] - trans_s[0];
        residual[1] = INNER_PRODUCT(affi_m[1], (affi_m[4] - T(1)), affi_m[7], 
                                    delta_2(0), delta_2(1), delta_2(2)) + trans_m[1] - trans_s[1];
        residual[2] = INNER_PRODUCT(affi_m[2], affi_m[5], (affi_m[8] - T(1)), 
                                    delta_2(0), delta_2(1), delta_2(2)) + trans_m[2] - trans_s[2];

        for (size_t i = 0; i < 3; i ++) {
            residual[i] = _coeff * residual[i];
        }

        return true;
    }

private:
    double _coeff;
    Eigen::Vector3d _master;
    Eigen::Vector3d _slave;
};


struct FitFunctor {
    FitFunctor(double coeff, Eigen::Vector3d point, Eigen::Vector3d mass_center, cv::Mat* depth_map)
        : _coeff(coeff), _point(point), _mass_center(mass_center), _depth_map(depth_map){
    }

    template <typename T>
    bool operator()(const T *const plane_paras, const T *const rigid_rot, const T *const rigid_trans, 
                    const T *const affi_rot, const T *const affi_trans, 
                    const T *const w, T *residual) const {
        T residual_1[3], residual_2[3];
        
        Eigen::Vector3d local_trans;
        local_trans[0] = INNER_PRODUCT(affi_rot[0], affi_rot[3], affi_rot[6], _point(0), _point(1), _point(2))
        + affi_trans[0];
        local_trans[0] = INNER_PRODUCT(affi_rot[1], affi_rot[4], affi_rot[7], _point(0), _point(1), _point(2))
        + affi_trans[1];
        local_trans[0] = INNER_PRODUCT(affi_rot[2], affi_rot[5], affi_rot[8], _point(0), _point(1), _point(2))
        + affi_trans[2];
        
        T theta = rigid_rot[0];
        T x = rigid_rot[1];
        T y = rigid_rot[2];
        T z = sqrt(T(1.0) - x * x - y * y);
        T r[9]; // rotation matrix in axis-angle form
        r[0] = cos(theta) + (T(1) - cos(theta)) * x * x;
        r[1] = (T(1) - cos(theta)) * y * x + sin(theta) * z;
        r[2] = (T(1) - cos(theta)) * z * x - sin(theta) * y;
        r[3] = (T(1) - cos(theta)) * x * y - sin(theta) * z;
        r[4] = cos(theta) + (T(1) - cos(theta)) * y * y;
        r[5] = (T(1) - cos(theta)) * z * y + sin(theta) * x;
        r[6] = (T(1) - cos(theta)) * x * z + sin(theta) * y;
        r[7] = (T(1) - cos(theta)) * y * z - sin(theta) * x;
        r[8] = cos(theta) + (T(1) - cos(theta)) * z * z;

        Eigen::Vector3d delta_1 = local_trans - _mass_center;
        // lack of correspondences, not finished yet
        residual_1[0] = INNER_PRODUCT(r[0], r[3], r[6], delta_1(0), 
                                    delta_1(1), delta_1(2)) + _mass_center(0) + rigid_trans[0];
        residual_1[1] = INNER_PRODUCT(r[1], r[4], r[7], delta_1(0), 
                                    delta_1(1), delta_1(2)) + _mass_center(1) + rigid_trans[1];
        residual_1[2] = INNER_PRODUCT(r[2], r[5], r[8], delta_1(0), 
                                    delta_1(1), delta_1(2)) + _mass_center(2) + rigid_trans[2];
          
        // see PointCloud::load for the same convertion                            
        const int scale = 100;  // scale the raw data
        float constant = 575.8; // kinect focal length: 575.8                                    
        
        // Here is a simple version about (u, v) map because I haven't implement the WLS reconstruction
        // Since the call of at function, only numerical differentiation can be used
        T depth = T(_depth_map->at<float>(int(plane_paras[0]+T(0.5)), int(plane_paras[1]+T(0.5)))) 
        * T(0.001) * T(scale); // mm -> m
        T u = (plane_paras[1] - T(_depth_map->cols) / T(2)) * depth / T(constant); 
        T v = (T(_depth_map->rows) / T(2) - plane_paras[0]) * depth / T(constant);
        
        // have not added rigid rigid_transformation here
        residual_2[0] = u;
        residual_2[1] = v;
        residual_2[2] = depth;
        
//         Eigen::Vector3d delta_2;
//         delta_2(0) = u - _mass_center(0);
//         delta_2(1) = v - _mass_center(1);
//         delta_2(2) = depth - _mass_center(2);
//         
//         residual_2[0] = INNER_PRODUCT(r[0], r[3], r[6], delta_2(0), 
//                                     delta_2(1), delta_2(2)) + _mass_center(0) + rigid_trans[0];
//         residual_2[1] = INNER_PRODUCT(r[1], r[4], r[7], delta_2(0), 
//                                     delta_2(1), delta_2(2)) + _mass_center(1) + rigid_trans[1];
//         residual_2[2] = INNER_PRODUCT(r[2], r[5], r[8], delta_2(0), 
//                                     delta_2(1), delta_2(2)) + _mass_center(2) + rigid_trans[2];
        
        for (size_t i = 0; i < 3; i ++) {
            residual[i] = _coeff * w[0] * (residual_1[i] - residual_2[i]);
        }

        return true;
    }

private:
    double _coeff;
    Eigen::Vector3d _point;
    Eigen::Vector3d _mass_center;
    cv::Mat* _depth_map;
};


struct ConfFunctor {
    ConfFunctor(double coeff)
        : _coeff(coeff) {
    }

    template <typename T>
    bool operator()(const T *const w, T *residual) const {
        residual[0] = _coeff * (T(1.0) - w[0] * w[0]);
        return true;
    }

private:
    double _coeff;
};


class Solver
{
public:
    typedef ceres::Problem Problem;
    typedef ceres::Solver::Options Options;
    typedef ceres::CostFunction CostFunction;
    typedef ceres::Solver::Summary Summary;

public:
    Solver(PointCloud *source, PointCloud* target);
    ~Solver();
    
    void initForTest();
    void initParameters();
    void buildProblem();
    void setOptions(); 
    
    void step();
    void apply();
    
    inline PointCloud* getSource(){
        return source_;
    }
    
    inline PointCloud* getTarget(){
        return target_;
    }
    
    void printParameters();

private:
    PointCloud *source_;
    PointCloud *target_;

    double rigid_alpha_;
    double smooth_alpha_;
    double fit_alpha_;
    double conf_alpha_;

    Problem problem_;
    Options options_;
    Summary summary_;
};
#endif //SOLVER_H
