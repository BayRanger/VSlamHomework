//
// Created by 高翔 on 2017/12/15.
//
#include <iostream>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

double fun(double a, double b, double c, double x)
{
    return exp(a * x * x + b * x + c);
}

int main(int argc, char **argv) {
    double ar = 10.0, br = 20.0, cr = 10.0;         // 真实参数值
    double ae = 20.0, be = -10.0, ce = 50.0;        // 估计参数值
    int N = 100;                                 // 数据点
    double w_sigma = 1.0;                        // 噪声Sigma值
    cv::RNG rng;                                 // OpenCV随机数产生器

    vector<double> x_data, y_data;      // 数据
    for (int i = 0; i < N; i++) {
        double x = i / 100.0;
        x_data.push_back(x);
        y_data.push_back(exp(ar * x * x + br * x + cr) + rng.gaussian(w_sigma));
    }

    // 开始Gauss-Newton迭代
    int iterations = 100;    // 迭代次数
    double cost = 0, lastCost = 0;  // 本次迭代的cost和上一次迭代的cost

    for (int iter = 0; iter < iterations; iter++) {

        Matrix3d H = Matrix3d::Zero();             // Hessian = J^T J in Gauss-Newton
        Vector3d b = Vector3d::Zero();             // bias
        cost = 0;

        for (int i = 0; i < N; i++) {
            double xi = x_data[i], yi = y_data[i];  // 第i个数据点
            // start your code here
            double error = fun(ae,be,ce,xi) - yi;   // 第i个数据点的计算误差
            //error = 0; // 填写计算error的表达式
            Vector3d J; // 雅可比矩阵
            J[0] =  fun(ae,be,ce,xi)*xi*xi;  // de/da
            J[1] = fun(ae,be,ce,xi)*xi;  // de/db
            J[2] = fun(ae,be,ce,xi);  // de/dc

            H += J * J.transpose(); // GN近似的H
            b += -error * J;
            // end your code here

            cost += error * error;
        }

        // 求解线性方程 Hx=b，建议用ldlt
 	// start your code here
        Vector3d dx;
        dx =H.inverse()*b;
	// end your code here

        if (isnan(dx[0])) {
            cout << "result is nan!" << endl;
            break;
        }

        if (iter > 0 && cost > lastCost) {
            // 误差增长了，说明近似的不够好
            cout << "cost: " << cost << ", last cost: " << lastCost << endl;
            break;
        }

        // 更新abc估计值
        ae += dx[0];
        be += dx[1];
        ce += dx[2];

        lastCost = cost;

        cout << "total cost: " << cost << endl;
    }

    cout << "estimated abc = " << ae << ", " << be << ", " << ce << endl;
    return 0;
}