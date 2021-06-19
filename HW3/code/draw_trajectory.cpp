#include <sophus/se3.hpp>
#include <string>
#include <iostream>
#include <fstream>
#include <unistd.h>
#include <sstream>
/*
STL containers take an optional template parameter, 
the allocator type. When using STL containers on 
fixed-size vectorizable Eigen types, you need tell 
the container to use an allocator that will always 
allocate memory at 16-byte-aligned (or more) locations.
 Fortunately, Eigen does provide such an allocator: Eigen::aligned_allocator.


*/
// need pangolin for plotting trajectory
#include <pangolin/pangolin.h>

using namespace std;
using SE3Vec =vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>>;
// path to trajectory file

string trajectory_file = "../trajectory.txt";
string gt_file = "../groundtruth.txt";
string et_file="../estimated.txt";
void DrawTrajectory(vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>>);
void DrawTrajectory(const SE3Vec &gt, const SE3Vec &esti);
bool extractPoseVec(string filename,SE3Vec& v)
{
    ifstream file(filename);
    if(!file.good())
    {
        cout<<"[ERROR]; Invalid data file"<<endl;
        return false;
    }
    SE3Vec trajectory;
    string str;
    while (std::getline(file, str))
    {
        double _,tx,ty,tz,qx,qy,qz,qw;
        istringstream iss(str);
        iss>> _>>tx>>ty>>tz>>qx>>qy>>qz>>qw;
        Sophus::SE3d p1(Eigen::Quaterniond(qx,qy,qz,qw),Eigen::Vector3d(tx,ty,tz));
        v.push_back(p1);
        
    }
    cout<<"be true"<<endl;
    return true;

}
// function for plotting trajectory, don't edit this code
// start point is red and end point is blue

//SE3Vec getTragectory()
int main(int argc, char **argv) {
  
    SE3Vec trajectory,gt,et;
    if(!extractPoseVec(trajectory_file, trajectory))
    {
        return -1;
    }
        if(!extractPoseVec(et_file, et))
    {
        return -1;
    }
        if(!extractPoseVec(gt_file, gt))
    {
        return -1;
    }
    //DrawTrajectory(trajectory);
    DrawTrajectory(et,gt);
     return 0;
}

/*******************************************************************************************/
void DrawTrajectory(vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>> poses) {
    if (poses.empty()) {
        cerr << "Trajectory is empty!" << endl;
        return;
    }

    // create pangolin window and plot the trajectory
    pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
            pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
    );

    pangolin::View &d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));


    while (pangolin::ShouldQuit() == false) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        glLineWidth(2);
        for (size_t i = 0; i < poses.size() - 1; i++) {
            glColor3f(1 - (float) i / poses.size(), 0.0f, (float) i / poses.size());
            glBegin(GL_LINES);
            auto p1 = poses[i], p2 = poses[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }
        //pangolin::SaveFramebuffer("heee", );
        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }

}

void DrawTrajectory(const SE3Vec &gt, const SE3Vec &esti) {
    // create pangolin window and plot the trajectory
    pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
            pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
    );

    pangolin::View &d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));


    while (pangolin::ShouldQuit() == false) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        glLineWidth(2);
        for (size_t i = 0; i < gt.size() - 1; i++) {
            glColor3f(0.0f, 0.0f, 1.0f);  // blue for ground truth
            glBegin(GL_LINES);
            auto p1 = gt[i], p2 = gt[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }

        for (size_t i = 0; i < esti.size() - 1; i++) {
            glColor3f(1.0f, 0.0f, 0.0f);  // red for estimated
            glBegin(GL_LINES);
            auto p1 = esti[i], p2 = esti[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }
        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }
}
 