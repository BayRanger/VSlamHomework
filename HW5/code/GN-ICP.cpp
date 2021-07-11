#include <iostream>
#include <fstream>
#include <string>
#include <vector>

#include "sophus/se3.hpp"


using namespace std;

typedef vector<Sophus::SE3d> SE3dVec; 

void fileToSE3dvec(string filename, SE3dVec& src_pts, SE3dVec& gt_pts)
{
    ifstream file;
    file.open(filename);
    std::string line;
    while(getline(file,line)) {
        istringstream iss(line);
        double linedata[16];
        for (int i =0;i<16;i++) {
            iss>>linedata[i];
        }
        Eigen::Quaterniond  src_quat(linedata[7],linedata[4],linedata[5],linedata[6]);
        Eigen::Vector3d src_tran(linedata[1],linedata[2],linedata[3]);
        src_pts.emplace_back(src_quat,src_tran);
        
        Eigen::Quaterniond  gt_quat(linedata[15],linedata[12],linedata[13],linedata[14]);
        Eigen::Vector3d gt_tran(linedata[9],linedata[10],linedata[11]);
        gt_pts.emplace_back(gt_quat,gt_tran);

    }

    file.close();

}

int main()
{
    SE3dVec gt_vec,src_vec;
    string filename="../compare.txt";
    fileToSE3dvec(filename,src_vec,gt_vec);
    assert(gt_vec.size() == src_vec.size());
    cout<<"data size "<<gt_vec.size()<<endl;
}

/**
 * float is a 32 bit IEEE 754 single precision Floating Point Number1 bit for the sign, (8 bits for the exponent, and 23* for the value), i.e. 
 * float has 7 decimal digits of precision.
 * double is a 64 bit IEEE 754 double precision Floating Point Number (1 bit for the sign, 11 bits for the exponent, and 52* bits for the value), 
 * i.e. double has 15 decimal digits of precision.
 * 
 */