#include <iostream>
#include <fstream>
#include <string>
#include <vector>

#include "sophus/se3.hpp"


using namespace std;

typedef vector<Sophus::SE3d> SE3dVec; 

void fileToSE3dvec(string filename, SE3dVec& tgt_pts, SE3dVec& src_pts)
{
    ifstream file;
    file.open(filename);
    std::string line;
    while(getline(file,line)) {
        //std::cout<<line<<endl;
        istringstream iss(line);
        double linedata[15];
        //double tmp_a,tmp_b,tmp_c;
        //iss>>*linedata;
        for (int i =0;i<15;i++) {
            iss>>linedata[i];
        }
        cout<<cout.precision(15)<<linedata[0]<<"   "<<linedata[1]<<endl;

    }

    file.close();

}

int main()
{
    SE3dVec data1,data2;
    string filename="../compare.txt";
    fileToSE3dvec(filename,data1,data2 );

}

/**
 * float is a 32 bit IEEE 754 single precision Floating Point Number1 bit for the sign, (8 bits for the exponent, and 23* for the value), i.e. 
 * float has 7 decimal digits of precision.
 * double is a 64 bit IEEE 754 double precision Floating Point Number (1 bit for the sign, 11 bits for the exponent, and 52* bits for the value), 
 * i.e. double has 15 decimal digits of precision.
 * 
 */