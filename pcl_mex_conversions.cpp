#include "mex.h"
#include <Eigen/Core>
#include <point_cloud.h>
#include <pcl/point_types.h>

#include "pcl_mex_conversions.h"


void mxArrayToPointCloud(const mxArray *mxIn,
        pcl::PointCloud<pcl::PointNormal>::Ptr cloud) {
    mwSize nrows = mxGetM(mxIn);
    mwSize ncols = mxGetN(mxIn);

    double *inMatrix;
    inMatrix = mxGetPr(mxIn);

    Eigen::MatrixXd inMatrixEigen(Eigen::Map<Eigen::MatrixXd> (
            inMatrix, nrows, ncols));

    if (ncols != 3) {
        mexErrMsgIdAndTxt("PCLMex:mxArrayToPointCloud:not3Columns", "Pointcloud must be Mx3 matrix.");
    }

    for (size_t i = 0; i < inMatrixEigen.rows(); i++) {
        pcl::PointNormal temp;
        temp.x = inMatrixEigen(i, 0);
        temp.y = inMatrixEigen(i, 1);
        temp.z = inMatrixEigen(i, 2);
        temp.normal_x = inMatrixEigen(i, 3);
        temp.normal_y = inMatrixEigen(i, 4);
        temp.normal_z = inMatrixEigen(i, 5);
        cloud->push_back(temp);
    }
}
mxArray* pointCloudToMxArray(pcl::PointCloud<pcl::PointNormal>::Ptr cloud) {

    Eigen::MatrixXf outMatrixXf(cloud->getMatrixXfMap());

    Eigen::MatrixXd outMatrixXd = outMatrixXf.cast<double>();

    Eigen::MatrixXd outMatrixXdCleaned = outMatrixXd.block(0, 0,
        outMatrixXd.rows() - 1, outMatrixXd.cols());

    outMatrixXdCleaned.transposeInPlace();

    /* create the output matrix */
    mxArray* mxOut = mxCreateDoubleMatrix((mwSize) outMatrixXdCleaned.rows(),
        (mwSize) outMatrixXdCleaned.cols(), mxREAL);

    /* get a pointer to the real data in the output matrix */
    double *outMatrix = mxGetPr(mxOut);

    Eigen::Map<Eigen::MatrixXd>( outMatrix,
        outMatrixXdCleaned.rows(), outMatrixXdCleaned.cols() ) =
      outMatrixXdCleaned;
    return mxOut;
}

mxArray* matrix4fToMxArray(Eigen::Matrix4f matrix4f) {

    Eigen::MatrixXf outMatrixXf(matrix4f);

    Eigen::MatrixXd outMatrixXd = outMatrixXf.cast<double>();

    Eigen::MatrixXd outMatrixXdCleaned = outMatrixXd.block(0, 0,
        outMatrixXd.rows() - 1, outMatrixXd.cols());

    outMatrixXdCleaned.transposeInPlace();

    /* create the output matrix */
    mxArray* mxOut = mxCreateDoubleMatrix((mwSize) outMatrixXdCleaned.rows(),
        (mwSize) outMatrixXdCleaned.cols(), mxREAL);

    /* get a pointer to the real data in the output matrix */
    double *outMatrix = mxGetPr(mxOut);

    Eigen::Map<Eigen::MatrixXd>( outMatrix,
        outMatrixXdCleaned.rows(), outMatrixXdCleaned.cols() ) =
      outMatrixXdCleaned;
    return mxOut;
}
