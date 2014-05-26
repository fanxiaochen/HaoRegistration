#ifndef RANGE_IMAGE_H
#define RANGE_IMAGE_H

#include <pcl/range_image/range_image.h>

#include "point_cloud.h"

class RangeImage: public pcl::RangeImage
{
public:
    RangeImage();
    virtual ~RangeImage();

    void getPointCloud();
private:
    boost::shared_ptr<PointCloud> point_cloud_;

//   Eigen::Vector2d parameters_;

};

#endif //RANGE_IMAGE_H
