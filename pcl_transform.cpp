// c system headers
#include <point_cloud.h>
#include <pcl/common/transforms.h>
#include <Eigen/Core>

// cpp system headers
#include <iostream>

// other headers
#include "mex.h"
#include "./pcl_mex_conversions.h"

// icp headers
//#include <pcl/features/normal_3d.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>
#include <pcl/filters/filter.h>

//convenient typedefs
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

/* The gateway function */
void mexFunction(int nlhs, mxArray *plhs[],
                 int nrhs, const mxArray *prhs[]) {
    double *inMatrix1;               /* Mx3 input matrix */
    double *inMatrix2;               /* Mx3 input matrix */
    double *outMatrix;              /* output matrix */

    /* check for proper number of arguments */
    if (nrhs != 2) {
        mexErrMsgIdAndTxt("MyToolbox:arrayProduct:nrhs", "Two inputs required.");
    }
    if (nlhs != 1) {
        mexErrMsgIdAndTxt("MyToolbox:arrayProduct:nlhs", "One output required.");
    }

    /* make sure the second input argument is type double */
    //if ( !mxIsDouble(prhs[1]) ||
    //     mxIsComplex(prhs[1])) {
    //    mexErrMsgIdAndTxt("MyToolbox:arrayProduct:notDouble", "Input matrix must be type double.");
    //}

    /* check that number of columns in first input argument is 1 */
    if (mxGetN(prhs[0]) != 6) {
        mexErrMsgIdAndTxt("MyToolbox:arrayProduct:not6Columns", "Input must have 6 columns.");
    }
    if (mxGetN(prhs[1]) != 6) {
        mexErrMsgIdAndTxt("MyToolbox:arrayProduct:not6Columns", "Input must have 6 columns.");
    }

    /* create a pointer to the real data in the input matrix  */
    inMatrix1 = mxGetPr(prhs[0]);
    inMatrix2 = mxGetPr(prhs[1]);
    //inTransform = mxGetPr(prhs[1]);

    /* get dimensions of the input matrix */
    mwSize nrows1 = mxGetM(prhs[0]);
    mwSize nrows2 = mxGetM(prhs[1]);

    pcl::PointCloud<pcl::PointNormal>::Ptr src (new pcl::PointCloud<pcl::PointNormal> ());
    mxArrayToPointCloud(prhs[0], src);

    pcl::PointCloud<pcl::PointNormal>::Ptr tgt (new pcl::PointCloud<pcl::PointNormal> ());
    mxArrayToPointCloud(prhs[1], tgt);

    //remove NAN points from the cloud
    //std::vector<int> indices;
    //pcl::removeNaNFromPointCloud(*m.cloud,*m.cloud, indices);
    
    
    pcl::IterativeClosestPointWithNormals<pcl::PointNormal, pcl::PointNormal> reg;
    reg.setTransformationEpsilon (1e-6);
    // Set the maximum distance between two correspondences (src<->tgt) to 10cm
    // Note: adjust this based on the size of your datasets
    reg.setMaxCorrespondenceDistance (0.1);
    reg.setRANSACOutlierRejectionThreshold (0.02);

    // Not implemented in version on cluster (2019) https://pointclouds.org/documentation/classpcl_1_1_iterative_closest_point_with_normals.html
    //reg.setEnforceSameDirectionNormals(true);
    //reg.setUseSymmetricObjective(true);

    reg.setInputSource (src);
    reg.setInputTarget (tgt);


    //
    // Run the same optimization in a loop and visualize the results
    Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev, targetToSource;
    PointCloudWithNormals::Ptr reg_result = src;
    reg.setMaximumIterations (2);


    for (int i = 0; i < 30; ++i)
    {
        //PCL_INFO ("Iteration Nr. %d.\n", i);

        // save cloud for visualization purpose
        src = reg_result;

        // Estimate
        reg.setInputSource (src);
        reg.align (*reg_result);

        //accumulate transformation between each Iteration
        Ti = reg.getFinalTransformation () * Ti;


        //if the difference between this transformation and the previous one
        //is smaller than the threshold, refine the process by reducing
        //the maximal correspondence distance
        if (fabs ((reg.getLastIncrementalTransformation () - prev).sum ()) < reg.getTransformationEpsilon ())
          reg.setMaxCorrespondenceDistance (reg.getMaxCorrespondenceDistance () - 0.001);

        prev = reg.getLastIncrementalTransformation ();

    }

    //
    // Get the transformation from target to source
    targetToSource = Ti.inverse();

    plhs[0] = matrix4fToMxArray(targetToSource);

}
