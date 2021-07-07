#pragma once

void mxArrayToPointCloud(const mxArray *mxIn,
        pcl::PointCloud<pcl::PointNormal>::Ptr cloud);

mxArray* pointCloudToMxArray(pcl::PointCloud<pcl::PointNormal>::Ptr cloud);

mxArray* matrix4fToMxArray(Eigen::Matrix4f matrix4f);