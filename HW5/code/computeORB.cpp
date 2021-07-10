//
// Created by 高翔 on 2017/12/19.
// 本程序演示ORB是如何提取、计算和匹配的
//

#include <opencv2/opencv.hpp>

#include <string>
#include <math.h>       /* sin */
#include "orb_pattern.h"
#define PI 3.14159265

using namespace std;

// global variables
string first_file = "../1.png";
string second_file = "../2.png";

const double pi = 3.1415926;    // pi


// TODO implement this function
/**
 * compute the angle for ORB descriptor
 * @param [in] image input image
 * @param [in|out] detected keypoints
 */
void computeAngle(const cv::Mat &image, vector<cv::KeyPoint> &keypoints);
// TODO implement this function
/**
 * compute ORB descriptor
 * @param [in] image the input image
 * @param [in] keypoints detected keypoints
 * @param [out] desc descriptor
 */
typedef vector<bool> DescType;  // type of descriptor, 256 bools
int HammingDist(const DescType& desc1, const DescType& desc2);
void computeORBDesc(const cv::Mat &image, vector<cv::KeyPoint> &keypoints, vector<DescType> &desc);

// TODO implement this function
/**
 * brute-force match two sets of descriptors
 * @param desc1 the first descriptor
 * @param desc2 the second descriptor
 * @param matches matches of two images
 */
void bfMatch(const vector<DescType> &desc1, const vector<DescType> &desc2, vector<cv::DMatch> &matches);

int main(int argc, char **argv) {

    // load image
    cv::Mat first_image = cv::imread(first_file, 0);    // load grayscale image
    cv::Mat second_image = cv::imread(second_file, 0);  // load grayscale image

    // plot the image
    cv::imshow("first image", first_image);
    cv::imshow("second image", second_image);
    cv::waitKey(0);

    // detect FAST keypoints using threshold=40
    vector<cv::KeyPoint> keypoints;
    cv::FAST(first_image, keypoints, 40);
    cout << "keypoints: " << keypoints.size() << endl;

    // compute angle for each keypoint
    computeAngle(first_image, keypoints);

    // compute ORB descriptors
    vector<DescType> descriptors;
    computeORBDesc(first_image, keypoints, descriptors);

    // plot the keypoints
    cv::Mat image_show;
    cv::drawKeypoints(first_image, keypoints, image_show, cv::Scalar::all(-1),
                      cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    cv::imshow("features", image_show);
    cv::imwrite("feat1.png", image_show);
    cv::waitKey(0);

    // we can also match descriptors between images
    // same for the second
    vector<cv::KeyPoint> keypoints2;
    cv::FAST(second_image, keypoints2, 40);
    cout << "keypoints: " << keypoints2.size() << endl;

    // compute angle for each keypoint
    computeAngle(second_image, keypoints2);

    // compute ORB descriptors
    vector<DescType> descriptors2;
    computeORBDesc(second_image, keypoints2, descriptors2);

    // find matches
    vector<cv::DMatch> matches;
    std::cout<<descriptors.size()<<" " <<descriptors2.size() << std::endl;
    bfMatch(descriptors, descriptors2, matches);
    cout << "matches: " << matches.size() << endl;

    // plot the matches
    cv::drawMatches(first_image, keypoints, second_image, keypoints2, matches, image_show);
    cv::imshow("matches", image_show);
    cv::imwrite("matches.png", image_show);
    cv::waitKey(0);

    cout << "done." << endl;
    return 0;
}

// -------------------------------------------------------------------------------------------------- //

// compute the angle
void computeAngle(const cv::Mat &image, vector<cv::KeyPoint> &keypoints) {
    int half_patch_size = 8;
    for (auto &kp : keypoints) {
	// START YOUR CODE HERE (~7 lines)
        if ((kp.pt.x-half_patch_size)<0 ||(kp.pt.y-half_patch_size)<0 ||
        (kp.pt.x+half_patch_size)>=image.cols||(kp.pt.x+half_patch_size)>=image.rows)
        {
            continue;
        }
        //cout<<"not contine";
    float m10(0),m00(0),m01(0);
 
    for(int i=0;i<4*half_patch_size;i++)
    {
        int pos_x = half_patch_size*sin(i*PI/(2*half_patch_size));
        int pos_y = half_patch_size*cos(i*PI/(2*half_patch_size));
        m10+=(pos_y)*image.at<uchar>(kp.pt.y+pos_y,kp.pt.x+pos_x);
        m00+=image.at<uchar>(kp.pt.y+pos_y,kp.pt.x+pos_x);
        m01+=(pos_x)*image.at<uchar>(kp.pt.y+pos_y,kp.pt.x+pos_x);
    }
    float cx = m10/m00;
    float cy = m01/m00;
    kp.angle = atan2(cy,cx);
    // static int id =0;
    // kp.class_id =id;
    // id++;
    //cout<<cx<<" "<<cy<<" "<<kp.angle<<endl;

    }
    return;
}

// -------------------------------------------------------------------------------------------------- //
// ORB pattern

// compute the descriptor
void computeORBDesc(const cv::Mat &image, vector<cv::KeyPoint> &keypoints, vector<DescType> &desc) {
    for (auto &kp: keypoints) {
        DescType d(256, false);
        for (int i = 0; i < 256; i++) {
            int u_p = ORB_pattern[i*4];
            int v_p = ORB_pattern[i*4+1];
            int u_q = ORB_pattern[i*4+2];
            int v_q = ORB_pattern[i*4+3];
            int up_r = cos(kp.angle)*u_p - sin(kp.angle)*v_p + kp.pt.y;
            int vp_r = sin(kp.angle)*u_p + cos(kp.angle)* v_p +kp.pt.x; 
            int uq_r = cos(kp.angle)*u_q - sin(kp.angle)*v_q + kp.pt.y;
            int vq_r = sin(kp.angle)*u_q + cos(kp.angle)* v_q + kp.pt.x; 
            if(up_r<0||vp_r<0||uq_r<0||vq_r<0||
            up_r>=image.rows||uq_r>=image.rows||vp_r>=image.cols||vq_r>=image.cols)
            {
                d.clear();
                break;
            }


            // START YOUR CODE HERE (~7 lines)
            //std::cout<<"image size "<<image.rows<<" "<<image.cols<<std::endl;
            //std::cout<<up_r<<" "<<vp_r<<" "<<uq_r<<" "<<vq_r<<std::endl;
            d[i] = (image.at<uchar>(up_r,vp_r)>image.at<uchar>(uq_r,vq_r))?0:1;  // if kp goes outside, set d.clear()
	    // END YOUR CODE HERE
        }
        std::cout<<"dsize "<<d.size()<<std::endl;
        desc.push_back(d);
    }

    int bad = 0;
    for (auto &d: desc) {
        if (d.empty()) bad++;
    }
    cout << "bad/total: " << bad << "/" << desc.size() << endl;
    return;
}

int HammingDist(const DescType& desc1, const DescType& desc2)
{
    int rtn_val =0;
    for (int i =0; i<desc1.size(); i++)
    {
        if (desc1[i]!=desc2[i])
        {
        rtn_val++;
        }
    }
    return rtn_val;

}
// brute-force matching
void bfMatch(const vector<DescType> &desc1, const vector<DescType> &desc2, vector<cv::DMatch> &matches) {
    int d_max = 50;

    // START YOUR CODE HERE (~12 lines)
    // find matches between desc1 and desc2. 
    // END YOUR CODE HERE
    for (int i =0;i<desc1.size();i++) {
        int best_dist = d_max;
        int best_id = -1;
        if (!desc1[i].size())
        {
            continue;
        }
        for (int j =0; j<desc2.size();j++) {
            //std::cout<<desc1[i].size()<<"   "<<desc2[j].size()<<std::endl;    
            if (!desc2[j].size())
                {
                    continue;
                }
            int dist =HammingDist(desc1[i],desc2[j]);
            cout<<"hamming distance "<<dist<<endl;
            if (dist<best_dist)
                {
                    best_id = j;
                    best_dist = dist;
                }            
        }
        if(best_dist<d_max) {
            cout<<"add match"<<endl;
            matches.emplace_back(i,best_id,best_dist);
        }
    }

    for (auto &m: matches) {
        cout << m.queryIdx << ", " << m.trainIdx << ", " << m.distance << endl;
    }
    return;
}
