//
// Created by gaoxiang on 19-5-4.
//

#ifndef MYSLAM_VIEWER_H
#define MYSLAM_VIEWER_H

#include <thread>
#include <pangolin/pangolin.h>

#include "frame.h"
#include "slammap.h"

namespace MySlam {

/**
 * 可视化
 */
class Viewer {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Viewer> ptr;

    Viewer();

    void SetMap(SLAMMap::ptr map) { map_ = map; }

    void Close();

    // 增加一个当前帧
    void AddCurrentFrame(frame::ptr current_frame);

    // 更新地图
    void UpdateMap();

    void DrawFrame(frame::ptr frame, const float* color);
   private:
    void ThreadLoop();


    void DrawMapPoints();

    void FollowCurrentFrame(pangolin::OpenGlRenderState& vis_camera);

    /// plot the features in current frame into an image
    cv::Mat PlotFrameImage();

    frame::ptr current_frame_ = nullptr;
    SLAMMap::ptr map_ = nullptr;

    std::thread viewer_thread_;
    bool viewer_running_ = true;

    std::map<unsigned long, frame::ptr> active_keyframes_;
    std::map<unsigned long, mappoint::ptr> active_landmarks_;
    bool map_updated_ = false;

    std::mutex viewer_data_mutex_;
};
}  // namespace myslam

#endif  // MYSLAM_VIEWER_H
