//
// Created by 高翔 on 2017/12/19.
// 本程序演示如何从Essential矩阵计算R,t
//

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace Eigen;

#include <sophus/so3.hpp>

#include <iostream>

using namespace std;

int main(int argc, char **argv) {

    // 给定Essential矩阵
    Matrix3d E;
    E << -0.0203618550523477, -0.4007110038118445, -0.03324074249824097,
            0.3939270778216369, -0.03506401846698079, 0.5857110303721015,
            -0.006788487241438284, -0.5815434272915686, -0.01438258684486258;

    // 待计算的R,t
    Matrix3d R;
    Vector3d t;

    // SVD and fix sigular values
    // START YOUR CODE HERE
    JacobiSVD<MatrixXd> svd(E, ComputeThinU | ComputeThinV);
    double sigma = svd.singularValues()(0) + svd.singularValues()(1);
    sigma = sigma/2.0;
    DiagonalMatrix<double, 3> Sigma(sigma, sigma, 0);
    auto U = svd.matrixU();
    auto V = svd.matrixV();
    Matrix3d Rz  = AngleAxisd(0.5*M_PI, Vector3d::UnitZ()).matrix();
    Matrix3d Rminusz = AngleAxisd(-0.5*M_PI, Vector3d::UnitZ()).matrix();

    // END YOUR CODE HERE

    // set t1, t2, R1, R2 
    // START YOUR CODE HERE
    Matrix3d t_wedge1 = U*Rz*Sigma*U.transpose();
    Matrix3d t_wedge2 = U*Rminusz*Sigma*U.transpose();

    Matrix3d R1 = U*Rz.transpose()*V.transpose();
    Matrix3d R2 = U*Rminusz.transpose()*V.transpose();
    // END YOUR CODE HERE

    cout << "R1 = " << R1 << endl;
    cout << "R2 = " << R2 << endl;
    cout << "t1 = " << t_wedge1<<"\n-->"<<Sophus::SO3d::vee(t_wedge1).transpose() << endl;
    cout << "t2 = " << t_wedge2<<"\n-->"<< Sophus::SO3d::vee(t_wedge2).transpose() << endl;

    // check t^R=E up to scale
    Matrix3d tR = t_wedge1 * R1;
    cout << "t^R = " << tR << endl;

    return 0;
}