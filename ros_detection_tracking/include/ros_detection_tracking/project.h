#ifndef PROJECT_H
#define PROJECT_H
#include <ros/ros.h>
#include "ros/package.h"
#include <iostream>
#include <numeric>
#include <algorithm>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <termios.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include "pcl_ros/transforms.h"
#include <Eigen/Dense>
#include <ros/package.h>
//#include <pcl/kdtree/kdtree_flann.h>
//#include <pcl/kdtree/flann.h>
#include <opencv/cv.hpp>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

namespace DetectandTract{

    class projector{
        private:
        cv::Mat image_proj;
        image_transport::Publisher image_publisher;
        tf::TransformBroadcaster tr_br;

        //crop lidar points
        float maxX = 25.0, maxY = 6.0, minZ = -1.4;

        struct initial_parameters
        {
            /* data */
            std::string camera_topic;
            std::string lidar_topic;
            cv::Mat camtocam_mat;
            cv::Mat cameraIn;
            cv::Mat RT;
        }i_params;
        void projection_callback(const sensor_msgs::Image::ConstPtr &img, 
                                const sensor_msgs::PointCloud2::ConstPtr &pc);
        void initParams();
        void matrix_to_transfrom(Eigen::MatrixXf &matrix, tf::Transform & trans);

        public:
        projector();


    };



}

#endif