//
// Created by gaoxiang on 19-5-4.
//
#include "viewer.h"
#include "feature.h"
#include "frame.h"
#include <unistd.h>
#include <pangolin/pangolin.h>
#include "sophus/se3.hpp"
#include <opencv2/opencv.hpp>
#include <iostream>
namespace MySlam {

Viewer::Viewer() {
    viewer_thread_ = std::thread(std::bind(&Viewer::ThreadLoop, this));
}

void Viewer::Close() {
    viewer_running_ = false;
    viewer_thread_.join();
}

void Viewer::AddCurrentFrame(frame::ptr current_frame) {
    std::unique_lock<std::mutex> lck(viewer_data_mutex_);
    current_frame_ = current_frame;
}

void Viewer::UpdateMap() {
    std::unique_lock<std::mutex> lck(viewer_data_mutex_);
    assert(map_ != nullptr);
    active_keyframes_ = map_->getActiveFrames();
    active_landmarks_ = map_->getActiveMapPoints();
    map_updated_ = true;
}

void Viewer::ThreadLoop() {
    pangolin::CreateWindowAndBind("MySLAM", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState vis_camera(
        pangolin::ProjectionMatrix(1024, 768, 400, 400, 512, 384, 0.1, 1000),
        pangolin::ModelViewLookAt(0, -5, -10, 0, 0, 0, 0.0, -1.0, 0.0));

    // Add named OpenGL viewport to window and provide 3D Handler
    pangolin::View& vis_display =
        pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, 0.0, 1.0, -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(vis_camera));

    const float blue[3] = {0, 0, 1};
    const float green[3] = {0, 1, 0};

    while (!pangolin::ShouldQuit() && viewer_running_) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        vis_display.Activate(vis_camera);

        std::unique_lock<std::mutex> lock(viewer_data_mutex_);
        if (current_frame_) {
            DrawFrame(current_frame_, green);
            FollowCurrentFrame(vis_camera);

            cv::Mat img = PlotFrameImage();
            cv::imshow("image", img);
            cv::waitKey(1);
        }

        if (map_) {
            DrawMapPoints();
        }

        pangolin::FinishFrame();
        usleep(5000);
    }

}

cv::Mat Viewer::PlotFrameImage() {
    cv::Mat img_out;
    cv::cvtColor(current_frame_->m_leftImg, img_out, cv::COLOR_GRAY2BGR);
    for (size_t i = 0; i < current_frame_->m_leftKPs.size(); ++i) {
        if (current_frame_->m_leftKPs[i]->m_mapPt.lock()) {
            auto feat = current_frame_->m_leftKPs[i];
            cv::circle(img_out, feat->getPoint2f(), 2, cv::Scalar(0, 250, 0),
                       2);
        }
    }
    return img_out;
}

void Viewer::FollowCurrentFrame(pangolin::OpenGlRenderState& vis_camera) {
    Sophus::SE3d Twc = current_frame_->getPose().inverse();
    pangolin::OpenGlMatrix m(Twc.matrix());
    vis_camera.Follow(m, true);
}

void Viewer::DrawFrame(frame::ptr frame, const float* color) {
    Sophus::SE3d Twc = frame->getPose();
    const float sz = 1.0;
    const int line_width = 2.0;
    const float fx = 400;
    const float fy = 400;
    const float cx = 512;
    const float cy = 384;
    const float width = 1080;
    const float height = 768;

    glPushMatrix();

    Sophus::Matrix4f m = Twc.matrix().template cast<float>();
    cout<<m<<endl;
    cout<<"============="<<endl;
    glMultMatrixf((GLfloat*)m.data());

    if (color == nullptr) {
        glColor3f(1, 0, 0);
    } else
        glColor3f(color[0], color[1], color[2]);

    glLineWidth(line_width);
    glBegin(GL_LINES);
    glVertex3f(0, 0, 0);
    glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
    glVertex3f(0, 0, 0);
    glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
    glVertex3f(0, 0, 0);
    glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
    glVertex3f(0, 0, 0);
    glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);

    glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);
    glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);

    glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
    glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);

    glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
    glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);

    glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
    glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);

    glEnd();
    glPopMatrix();
}

void Viewer::DrawMapPoints() {
    const float red[3] = {1.0, 0, 0};
    const float blue[3] = {0 , 0, 1};
    for (auto& kf : active_keyframes_) {
        DrawFrame(kf.second, blue);
    }

    glPointSize(2);
    glBegin(GL_POINTS);
    for (auto& landmark : active_landmarks_) {
        auto pos = landmark.second->getEigenPose();
        glColor3f(red[0], red[1], red[2]);
        glVertex3d(pos[0]/100, pos[1]/100, pos[2]/100);
    }
    glEnd();
}

}  // namespace myslam
