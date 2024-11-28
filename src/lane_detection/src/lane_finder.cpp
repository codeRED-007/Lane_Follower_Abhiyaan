#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/float64.hpp"
#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"
#include <mlpack/core.hpp>
#include <vector>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Dense>
#include <cmath>
#include <numeric>
#include <unordered_map>
#include <queue>
#include <armadillo>

#include "sensor_msgs/msg/camera_info.hpp"
#include "dbscan_cuda.cuh"

using namespace std;
using namespace cv;
using std::placeholders::_1;

class LaneFollower : public rclcpp::Node {
public:
    LaneFollower() : Node("lane_follower") {   
        this->declare_parameter("bt_low", 120);
        this->declare_parameter("bt_high", 140);

        bt_low = this->get_parameter("bt_low").as_int();
        bt_high = this->get_parameter("bt_high").as_int();

        subscription = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera_forward/image_raw", 10, 
            std::bind(&LaneFollower::binary_thresholding, this, std::placeholders::_1));
        
        subscription_caminfo = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/camera_forward/camera_info", 10, 
            std::bind(&LaneFollower::camera_info_callback, this, _1));
        
        publisher_far = this->create_publisher<sensor_msgs::msg::PointCloud2>("/far_ipm", 10);
        publisher_near = this->create_publisher<sensor_msgs::msg::PointCloud2>("/near_ipm", 10);
        db_publisher = this->create_publisher<sensor_msgs::msg::Image>("/dbImage", 10);
        db_publisher2 = this->create_publisher<sensor_msgs::msg::Image>("/dbImage2", 10);
        lane_publisher = this->create_publisher<sensor_msgs::msg::Image>("/lane_image", 10);
        thresh_publisher = this->create_publisher<sensor_msgs::msg::Image>("/threshImage", 10);
        width_publisher = this->create_publisher<std_msgs::msg::Float64>("/lane_width",10);
    }
private:
    void binary_thresholding(const sensor_msgs::msg::Image::SharedPtr msg) {   
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        if (!cv_ptr) {
            RCLCPP_ERROR(this->get_logger(), "Failed to convert the image");
            return;
        }

        cv::Mat cv_image = cv_ptr->image, gray_image, thresholded_image;
        int rows = cv_image.rows;
        int cols = cv_image.cols;

        cv::medianBlur(cv_image, cv_image, 5);
        cv::cvtColor(cv_image, gray_image, cv::COLOR_BGR2GRAY); 

        // Mask out the top quarter of the image
        for (int y = 0; y < rows / 4; ++y) {
            for (int x = 0; x < cols; ++x) {
                gray_image.at<uchar>(y, x) = 0;
            }
        }

        cv::inRange(gray_image, bt_low, bt_high, thresholded_image);
        thresh_publisher->publish(*cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", thresholded_image).toImageMsg());

        std::vector<cv::Point> white_pixel_indices;

        // Apply DBSCAN
        int eps = 10;
        Graph graph(thresholded_image, eps);
        DBSCAN scanner(&graph, 0.1f, 10);
        scanner.identify_cluster();

        arma::Row<size_t> assignments(graph.size());

        for (int i = 0; i < graph.size(); ++i) {
            assignments[i] = scanner.label(i);
        }

        for (int i = 0; i < graph.size(); ++i) {
            cv::Point point;
            point.x = graph.node(i).y;
            point.y = graph.node(i).x;
            white_pixel_indices.push_back(point);
        }

        std::unordered_map<size_t, size_t> clusterSizes;
        for (size_t i = 0; i < assignments.n_elem; ++i) {
            if (assignments[i] != SIZE_MAX) {
                clusterSizes[assignments[i]]++;
            }
        }

        std::vector<std::pair<size_t, size_t>> sortedClusters(clusterSizes.begin(), clusterSizes.end());
        std::sort(sortedClusters.begin(), sortedClusters.end(), 
                  [](const std::pair<size_t, size_t>& a, const std::pair<size_t, size_t>& b) {
                      return b.second < a.second; 
                  });

        if (sortedClusters.size() < 2) return;

        size_t largestClusterID = sortedClusters[0].first;
        size_t SecondLargestClusterID = sortedClusters[1].first;

        std::vector<cv::Point> largestCluster, secondLargestCluster;
        for (size_t i = 0; i < assignments.n_elem; ++i) {
            if (assignments[i] == largestClusterID) {
                largestCluster.push_back(white_pixel_indices[i]);
            } else if (assignments[i] == SecondLargestClusterID) {
                secondLargestCluster.push_back(white_pixel_indices[i]);
            }
        }

        cv::Mat dbImage = cv::Mat::zeros(gray_image.rows, gray_image.cols, CV_8UC1);
        cv::Mat dbImage2 = cv::Mat::zeros(gray_image.rows, gray_image.cols, CV_8UC1);

        for (const auto& point : largestCluster) {
            dbImage.at<uchar>(point.y, point.x) = 255;
            dbImage2.at<uchar>(point.y, point.x) = 255;
        }

        for (const auto& point : secondLargestCluster) {
            dbImage2.at<uchar>(point.y, point.x) = 255;
        }

        db_publisher->publish(*cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", dbImage).toImageMsg());
        db_publisher2->publish(*cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", dbImage2).toImageMsg());
        cv::imshow("window", dbImage2);
        cv::waitKey(1);
        width_finder(largestCluster,secondLargestCluster);
    }

    void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        camera_info = *msg;
        cam_info_received = true;
    }



    void width_finder(const std::vector<cv::Point>& largestCluster, const std::vector<cv::Point>& secondLargestCluster) {
        
        int current_y = 0, prev_y = -1;
        bool closest_flag = false;

        double width = 0.0;
        int lx = 0, sx = 0;

       
        for (int j =secondLargestCluster.size() - 1 ; j>0 ; j--) {

            // //cout <<secondLargestCluster[j] <<" "<< largestCluster[j]<<endl;

            current_y = secondLargestCluster[j].y;
            if (current_y != 0 && prev_y !=0 && current_y == prev_y) continue;  
            if (current_y ==0 ) continue;
            for (int i =largestCluster.size() - 1 ; i>0 ; i--) {
                // if ((largestCluster[i].y <= secondLargestCluster[j].y + 5)&&(largestCluster[i].y >= secondLargestCluster[j].y - 5)) {

                if (abs(largestCluster[i].y-secondLargestCluster[j].y)<5) {
                       
                        lx = largestCluster[i].x;
                        sx = secondLargestCluster[j].x;
                        
                        if (abs(lx - sx) > 80) {
                            closest_flag = true;
                            cout<<"got points!"<<endl;
                            break;
                        }
                }
                 
            }
            if (closest_flag) break;
            prev_y = current_y;
        }


        if (closest_flag) {
            // Process points
            auto l = process_point(current_y, lx);
            auto s = process_point(current_y, sx);
            cout<<"ipm_points: "<<l.first<<" "<<l.second<<" "<<s.first<<" "<<s.second<<endl;
            width = std::sqrt(std::pow(l.first - s.first, 2) + std::pow(l.second - s.second, 2));

            // Add the new width to the sliding window
            if (width > 2 and width < 3.5) width_window.push_back(width);
            else if (width<2) width_window.push_back(2);
            else if (width>3.5) width_window.push_back(3.5);
            else return;

            if (width_window.size() > window_size) {
                width_window.pop_front(); // Remove the oldest width value if the window is full
            }

            // Calculate the sliding window average
            sliding_avg = std::accumulate(width_window.begin(), width_window.end(), 0.0) / width_window.size();
            
            // Now, sliding_avg contains the average width over the last `window_size` measurements
            std::cout << "Sliding average width: " << sliding_avg << std::endl;
            auto width_msg = std_msgs::msg::Float64();
            width_msg.data = 3;
            width_publisher->publish(width_msg);
        }
    }


    std::pair<double, double> process_point(int y, int x) {
            sensor_msgs::msg::PointCloud2 pub_pointcloud;
        auto cloud_msg = std::make_unique<pcl::PointCloud<pcl::PointXYZ>>();

        // Camera extrinsic parameters
        float roll = 0;
        float pitch = 0;
        float yaw = 0;
        float h = 0.8;

        // Pre-compute sin and cos values
        double cy = cos(yaw);
        double sy = sin(yaw);
        double cp = cos(pitch);
        double sp = sin(pitch);
        double cr = cos(roll);
        double sr = sin(roll);

        // Rotation matrix K (combining yaw, pitch, and roll)
        Eigen::Matrix3d K;
        K << cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr,
            sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr,
            -sp,     cp * sr,                cp * cr;

        // Normal vector to the ground plane (assuming flat ground)
        Eigen::Vector3d nor(0.0, 1.0, 0.0);

        // Calculate nc, the rotated normal vector
        Eigen::Vector3d nc = K * nor;

        // Inverse camera intrinsic matrix
        auto caminfo = this->camera_info.k; // assuming row-major order
        Eigen::Map<Eigen::Matrix<double,3,3,Eigen::RowMajor>> kin(caminfo.data());
        kin = kin.inverse().eval();

        // Convert the pixel coordinates (x, y) to homogeneous coordinates
        Eigen::Vector3d uv_hom(x, y, 1);

        // Map pixel coordinates to 3D camera ray
        Eigen::Vector3d kin_uv = kin * uv_hom;

        // Calculate the denominator for scaling (distance along the ray to the plane)
        double denom = kin_uv.dot(nc);
        pair<double,double> point;
        point.first = 0.0;
        point.second = 0.0;
        // Ensure denom is not zero to avoid division by zero
        if (denom != 0) {
            // Scale the ray by the height of the plane      
            point.first = h * kin_uv[2] / denom;
            point.second = -h * kin_uv[0] / denom;

       
        } else {
            std::cerr << "Denominator is zero, invalid projection for point (" << x << ", " << y << ")" << std::endl;
        }
        return point;
        
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr subscription_caminfo;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_far;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_near;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr db_publisher;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr db_publisher2;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr lane_publisher;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr thresh_publisher;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr width_publisher;
    std::deque<double> width_window;  
    const int window_size = 20;
    double sliding_avg = 0;
    sensor_msgs::msg::CameraInfo camera_info;
    bool cam_info_received = false;
    int bt_low;
    int bt_high;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LaneFollower>());
    rclcpp::shutdown();
    return 0;
}
