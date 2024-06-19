/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#ifndef FEATURE_MANAGER_H
#define FEATURE_MANAGER_H

#include <list>
#include <algorithm>
#include <vector>
#include <numeric>
using namespace std;

#include <eigen3/Eigen/Dense>
using namespace Eigen;

#include <ros/console.h>
#include <ros/assert.h>

#include "parameters.h"
#include "../utility/tic_toc.h"

class FeaturePerFrame
{
  public:
    FeaturePerFrame(const Eigen::Matrix<double, 7, 1> &_point, double td)
    {
        point.x() = _point(0);
        point.y() = _point(1);
        point.z() = _point(2);
        uv.x() = _point(3);
        uv.y() = _point(4);
        velocity.x() = _point(5); 
        velocity.y() = _point(6); 
        cur_td = td;
        is_stereo = false;
    }
    void rightObservation(const Eigen::Matrix<double, 7, 1> &_point)
    {
        pointRight.x() = _point(0);
        pointRight.y() = _point(1);
        pointRight.z() = _point(2);
        uvRight.x() = _point(3);
        uvRight.y() = _point(4);
        velocityRight.x() = _point(5); 
        velocityRight.y() = _point(6); 
        is_stereo = true;
    }
    double cur_td;
    Vector3d point, pointRight;
    Vector2d uv, uvRight;
    Vector2d velocity, velocityRight;
    bool is_stereo;
    Vector3d pre_point{-1, -1, 1};
    Vector2d pre_uv{-1, -1};
    double timestamp;

    FeaturePerFrame(const FeaturePerFrame& feature_per_frame) {
        this->cur_td = feature_per_frame.cur_td;
        this->point = feature_per_frame.point;
        this->pointRight = feature_per_frame.pointRight;
        this->uv = feature_per_frame.uv;
        this->uvRight = feature_per_frame.uvRight;
        this->velocity = feature_per_frame.velocity;
        this->velocityRight = feature_per_frame.velocityRight;
        this->is_stereo = feature_per_frame.is_stereo;
        this->pre_point = feature_per_frame.pre_point;
        this->pre_uv = feature_per_frame.pre_uv;
        this->timestamp = feature_per_frame.timestamp;
    }

    FeaturePerFrame& operator=(const FeaturePerFrame& feature_per_frame) {
        if (this == &feature_per_frame) {
            return *this; // 处理自我赋值情况
        }

        this->cur_td = feature_per_frame.cur_td;
        this->point = feature_per_frame.point;
        this->pointRight = feature_per_frame.pointRight;
        this->uv = feature_per_frame.uv;
        this->uvRight = feature_per_frame.uvRight;
        this->velocity = feature_per_frame.velocity;
        this->velocityRight = feature_per_frame.velocityRight;
        this->is_stereo = feature_per_frame.is_stereo;
        this->pre_point = feature_per_frame.pre_point;
        this->pre_uv = feature_per_frame.pre_uv;
        this->timestamp = feature_per_frame.timestamp;

        return *this;
    }
};

class FeaturePerId
{
  public:
    int feature_id;
    int start_frame;
    vector<FeaturePerFrame> feature_per_frame;
    int used_num;
    double estimated_depth;
    int solve_flag; // 0 haven't solve yet; 1 solve succ; 2 solve fail;
    int tri_init_flag = 0; // 0 haven't init yet; 1 init succ; 2 init fail;
    double pre_id = -1;
    Vector2d mc{0,0};
    static int dy_num;

    FeaturePerId(int _feature_id, int _start_frame)
        : feature_id(_feature_id), start_frame(_start_frame),
          used_num(0), estimated_depth(-1.0), solve_flag(0)
    {
    }

    int endFrame();
    int isDy() const {
//        if (pre_id >= 0 && feature_id < -1)
//            return 1;
        if (feature_id == -1)
            return 1;
        return 0;
    }

    FeaturePerId(const FeaturePerId& feature_per_id) {
        this->feature_id = feature_per_id.feature_id;
        this->start_frame = feature_per_id.start_frame;
        this->feature_per_frame = feature_per_id.feature_per_frame; // 这里涉及到FeaturePerFrame类的拷贝构造函数
        this->used_num = feature_per_id.used_num;
        this->estimated_depth = feature_per_id.estimated_depth;
        this->solve_flag = feature_per_id.solve_flag;
        this->tri_init_flag = feature_per_id.tri_init_flag;
        this->mc = feature_per_id.mc;
        this->pre_id = feature_per_id.pre_id;
    }
};


class FeatureManager
{
  public:
    FeatureManager(Matrix3d _Rs[]);

    void setRic(Matrix3d _ric[]);
    void clearState();
    int getFeatureCount();
    int getDyFeatureCount();
    int getDyFailFeatureCount(int frame_count = 0);
    bool filterDyFrame(FeaturePerId& dyFeaturePerId, Vector2d pre_mc);
    bool addFeatureCheckParallax(int frame_count, const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image, double td);
    bool addFeatureCheckParallax(int frame_count, const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image, double td, double timestamp);
    vector<pair<Vector3d, Vector3d>> getCorresponding(int frame_count_l, int frame_count_r);
    //void updateDepth(const VectorXd &x);
    void setDepth(const VectorXd &x);
    void setDyDepth(const VectorXd &x);
    void setMc(const vector<Vector2d> &x);
    void removeFailures();
    void clearDepth();
    VectorXd getDepthVector();
    VectorXd getDyDepthVector();
    vector<Vector2d> getMcVector();
    void triangulate(int frameCnt, Vector3d Ps[], Matrix3d Rs[], Vector3d tic[], Matrix3d ric[]);
    void triangulatePoint(Eigen::Matrix<double, 3, 4> &Pose0, Eigen::Matrix<double, 3, 4> &Pose1,
                            Eigen::Vector2d &point0, Eigen::Vector2d &point1, Eigen::Vector3d &point_3d);
    void initFramePoseByPnP(int frameCnt, Vector3d Ps[], Matrix3d Rs[], Vector3d tic[], Matrix3d ric[]);
    bool solvePoseByPnP(Eigen::Matrix3d &R_initial, Eigen::Vector3d &P_initial, 
                            vector<cv::Point2f> &pts2D, vector<cv::Point3f> &pts3D);
    void removeBackShiftDepth(Eigen::Matrix3d marg_R, Eigen::Vector3d marg_P, Eigen::Matrix3d new_R, Eigen::Vector3d new_P);
    void removeBack();
    void removeFront(int frame_count);
    void removeOutlier(set<int> &outlierIndex);
    void removeDyOutlier(set<int> &outlierIndex);
    list<FeaturePerId> feature;
    int last_track_num;
    double last_average_parallax;
    int new_feature_num;
    int long_track_num;
    bool addDyFeature(int _feature_id, int index, double pre_depth, Vector2d pre_mc);
    int cur_dy_num = 0;

  private:
    double compensatedParallax2(const FeaturePerId &it_per_id, int frame_count);
    const Matrix3d *Rs;
    Matrix3d ric[2];
};

#endif