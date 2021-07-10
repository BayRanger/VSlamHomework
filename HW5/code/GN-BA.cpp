//
// Created by xiang on 12/21/17.
//

#include <Eigen/Core>
#include <Eigen/Dense>

using namespace Eigen;

#include <vector>
#include <fstream>
#include <iostream>
#include <iomanip>

#include "sophus/se3.hpp"

using namespace std;

typedef vector<Vector3d, Eigen::aligned_allocator<Vector3d>> VecVector3d;
typedef vector<Vector2d, Eigen::aligned_allocator<Vector3d>> VecVector2d;
typedef Matrix<double, 6, 1> Vector6d;

string p3d_file = "../p3d.txt";
string p2d_file = "../p2d.txt";

int main(int argc, char **argv) {

    VecVector2d p2d;
    VecVector3d p3d;
    Matrix3d K;
    double fx = 520.9, fy = 521.0, cx = 325.1, cy = 249.7;
    K << fx, 0, cx, 0, fy, cy, 0, 0, 1;

    // load points in to p3d and p2d 
    // START YOUR CODE HERE
    ifstream p3dfile;
    p3dfile.open(p3d_file);
    std::string line;
    while(getline(p3dfile,line)) {
        cout<<line<<endl;
        istringstream iss(line);
        Vector3d point3d;
        double tmp_a,tmp_b,tmp_c;
        iss>>tmp_a>>tmp_b>>tmp_c;
        p3d.emplace_back(tmp_a,tmp_b,tmp_c);
    }
    p3dfile.close();

    ifstream p2dfile;
    p2dfile.open(p2d_file);
    //std::string line;
    while(getline(p2dfile,line)) {
        //cout<<line<<endl;
        istringstream iss(line);
        Vector2d point2d;
        double tmp_a,tmp_b;
        iss>>tmp_a>>tmp_b;
        p2d.emplace_back(tmp_a,tmp_b);
    }
    p2dfile.close();
    // END YOUR CODE HERE
    assert(p3d.size() == p2d.size());

    int iterations = 100;
    double cost = 0, lastCost = 0;
    int nPoints = p3d.size();
    cout << "points: " << nPoints << endl;

    Sophus::SE3d T_esti; // estimated pose

    for (int iter = 0; iter < iterations; iter++) {

        Matrix<double, 6, 6> H = Matrix<double, 6, 6>::Zero();
        Vector6d b = Vector6d::Zero();
        T_esti = Sophus::SE3d::exp(b);

        cost = 0;
        // compute cost
        for (int i = 0; i < nPoints; i++) {
            // compute cost for p3d[I] and p2d[I]
            // START YOUR CODE HERE 
            Eigen::Vector4d P_homo;
            //P_homo.block<3,1>(0,0) = VecVector3d[i];

            //Vector3d reproj_vec = K*T_esti.matrix()*VecVector3d[i];
            //Vector2d bias = p2d[i]-


	    // END YOUR CODE HERE

	    // compute jacobian
            Matrix<double, 2, 6> J;
            // START YOUR CODE HERE 

	    // END YOUR CODE HERE

            H += J.transpose() * J;
            //b += -J.transpose() * e;
        }

	// solve dx 
        Vector6d dx;

        // START YOUR CODE HERE 

        // END YOUR CODE HERE

        if (isnan(dx[0])) {
            cout << "result is nan!" << endl;
            break;
        }

        if (iter > 0 && cost >= lastCost) {
            // cost increase, update is not good
            cout << "cost: " << cost << ", last cost: " << lastCost << endl;
            break;
        }

        // update your estimation
        // START YOUR CODE HERE 

        // END YOUR CODE HERE
        
        lastCost = cost;

        cout << "iteration " << iter << " cost=" << cout.precision(12) << cost << endl;
    }

    cout << "estimated pose: \n" << T_esti.matrix() << endl;
    return 0;
}
