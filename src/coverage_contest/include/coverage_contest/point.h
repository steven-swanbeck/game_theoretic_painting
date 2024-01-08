#ifndef COVERAGE_CONTEST_POINT_H
#define COVERAGE_CONTEST_POINT_H

#define PCL_NO_PRECOMPILE
#include <pcl/point_types.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/search/kdtree.h>
#include <pcl/registration/transforms.h>
#include <pcl_ros/transforms.h>
#include <pcl/registration/icp_nl.h>

namespace Haphephobia {
    
    struct EIGEN_ALIGN16 Point {
        PCL_ADD_POINT4D;
        PCL_ADD_NORMAL4D;
        PCL_ADD_RGB;
        float intensity;
        float curvature;
        uint32_t t;
        uint16_t reflectivity;
        uint16_t ring;
        uint16_t ambient;
        uint32_t range;
        float sensor_a;
        float sensor_b;
        float sensor_c;
        float lidar_label;
        float semantic_label;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}

POINT_CLOUD_REGISTER_POINT_STRUCT(Haphephobia::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, normal_x, normal_x)
    (float, normal_y, normal_y)
    (float, normal_z, normal_z)
    (std::uint32_t, rgb, rgb)
    (float, intensity, intensity)
    (float, curvature, curvature)
    (std::uint32_t, t, t)
    (std::uint16_t, reflectivity, reflectivity)
    (std::uint16_t, ring, ring)
    (std::uint16_t, ambient, ambient)
    (std::uint32_t, range, range)
    (float, sensor_a, sensor_a)
    (float, sensor_b, sensor_b)
    (float, sensor_c, sensor_c)
    (float, lidar_label, lidar_label)
    (float, semantic_label, semantic_label)
)

typedef Haphephobia::Point PointT;
typedef pcl::PointCloud<PointT> PointCloud;

PCL_INSTANTIATE(NormalEstimationOMP, Haphephobia::Point);
PCL_INSTANTIATE(KdTree, Haphephobia::Point);
PCL_INSTANTIATE(transformPointCloud, Haphephobia::Point);
PCL_INSTANTIATE(IterativeClosestPointNonLinear, Haphephobia::Point);

#endif //COVERAGE_CONTEST_POINT_H
