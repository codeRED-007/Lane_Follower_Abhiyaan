#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <Eigen/Dense>
#include "cuda_runtime.h"
#include "rclcpp/rclcpp.hpp"
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include "opencv2/opencv.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/header.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include <math.h>
#include <unordered_map>
#include <utility>
#include "cv_bridge/cv_bridge.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#define BLOCKS 64
#define imin(a,b) (a<b?a:b)

using namespace std::chrono_literals;
using namespace std;
using namespace Eigen;
using std::placeholders::_1;
template<typename T>
__global__ void dev_matmul(const T *a, const T *b, T *output, int rows){
	//a is 3x3 matrix
	//b is 3x1 set of matrices
	//output is 3x1 set of matrices
	int thread_id= threadIdx.x;
	int block_id = blockIdx.x;
	int offset = block_id*(rows+BLOCKS-1)/BLOCKS + thread_id;

	if(offset < rows){
		#pragma unroll
		for(int i=0; i < 3; ++i){
			double temp=0;
			for(int k=0; k < 3; ++k){
				temp += a[i*3+k]*b[offset*3 + k];
			}
			output[offset*3 + i] = temp;
		}
	}

	//if(offset == 0){
		//printf("gpu_matmul");
		//for(int i=0;i < 3; ++i){
			//printf("%f ", output[i]);
		//}
		//printf("\n");
	//}

}



void matmul(double *a, double *b, double *c){
	//a is 3x3
	//b is 3x1
  	#pragma unroll
	for(int i=0; i < 3; ++i){
		double temp=0;
		for(int k=0; k < 3; ++k){
			temp += a[i*3 + k]*b[k]; 
		}
		c[i] = temp;
	}
}

struct pair_hash {
    template <class T1, class T2>
    std::size_t operator () (const std::pair<T1, T2>& pair) const {
        auto hash1 = std::hash<T1>{}(pair.first);
        auto hash2 = std::hash<T2>{}(pair.second);
        return hash1 ^ (hash2 << 1); // Combine the two hash values
    }
};

__global__ void dot(double* a, double* b, double* c, int rows) {
	int thread_id = threadIdx.x;
	int block_id = blockIdx.x;

	int offset = block_id*(rows+BLOCKS-1)/BLOCKS + thread_id;
	if(offset < rows){
		double temp=0;
 		#pragma unroll
		for(int i=0; i < 3; ++i){
		    temp += a[i]*b[offset*3+i];    
		}
		c[offset] = temp;
	}
	
	//debug
	//if(offset == 0){
		//for(int i=0;i < 3; ++i){
			//printf("gpu : %f ", c[0]);
		//}
		//printf("\n");
	//}

}

void downsample(cv::Mat& nonZeroCoordinates, cv::Mat& downsampled, int threshold = 5) {
    int gridSize = 5; // Adjust the grid size to control the level of downsampling

    // Step 1: Count the number of points in each grid cell
    std::unordered_map<std::pair<int, int>, int, pair_hash> gridCount;
    for (int i = 0; i < nonZeroCoordinates.rows; ++i) {
        cv::Point point = nonZeroCoordinates.at<cv::Point>(i);
        int gridX = point.x / gridSize;
        int gridY = point.y / gridSize;
        std::pair<int, int> gridCell(gridX, gridY);

        gridCount[gridCell]++;
    }

    // Step 2: Add points to the downsampled matrix if their grid cell meets the threshold
    std::unordered_map<std::pair<int, int>, cv::Point, pair_hash> gridMap;
    for (int i = 0; i < nonZeroCoordinates.rows; ++i) {
        cv::Point point = nonZeroCoordinates.at<cv::Point>(i);
        int gridX = point.x / gridSize;
        int gridY = point.y / gridSize;
        std::pair<int, int> gridCell(gridX, gridY);

        // Only add the point if the grid cell has enough points
        if (gridCount[gridCell] >= threshold && gridMap.find(gridCell) == gridMap.end()) {
            gridMap[gridCell] = point;
            downsampled.push_back(point);
        }
    }
}



//void log(cudaError_t &&error, int line=0){
	//std::cout << cudaGetErrorString(error) << "line : " << line << '\n' << std::flush;
//}

class IPM : public rclcpp::Node
{
  public:
    IPM()
    : Node("ipm"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
    {
	   //default values
	    this->declare_parameter("masked_image_topic","/dbImage");
	    this->declare_parameter("camera_topic","/camera_forward/camera_info");
	    this->declare_parameter("binary_threshold_low",100);
	    this->declare_parameter("binary_threshold_high",240);
	    this->declare_parameter("ipm_pitch",-24*M_PI/180);
	    this->declare_parameter("ipm_height",1.41);
	    this->declare_parameter("frame","map");


	    //getting useful params
	    this->get_parameter("masked_image_topic",masked_image_topic);
	    this->get_parameter("camera_topic",camera_topic);
	    ipm_pitch = this->get_parameter("ipm_pitch").as_double();
	    ipm_height = this->get_parameter("ipm_height").as_double();
	    binary_threshold_low = this->get_parameter("binary_threshold_low").as_int(); 
	    binary_threshold_high = this->get_parameter("binary_threshold_high").as_int();

	    frame = this->get_parameter("frame").as_string();
	//    cout<<masked_image_topic<<" "<<camera_topic<<endl;;
       subscription_caminfo = this->create_subscription<sensor_msgs::msg::CameraInfo>(camera_topic, 10, std::bind(&IPM::call, this, _1));
       subscription_img = this->create_subscription<sensor_msgs::msg::Image>(masked_image_topic, 10, std::bind(&IPM::process_img, this, _1));
       publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/igvc/ipm", 10);
       publisher_map = this->create_publisher<sensor_msgs::msg::PointCloud2>("/lanes", 10);

	   publisher_forward = this->create_publisher<sensor_msgs::msg::Image>("/igvc/lanes_binary/forward", 10);
	   publisher_image = this->create_publisher<sensor_msgs::msg::Image>("/ipm_image", 10);
	   


	//    parameter_callback_handle_ = this->add_on_set_parameters_callback(std::bind(&IPM::parameter_callback, this, std::placeholders::_1));
	   
    }

  private:
    void call(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        this->camera_info = *msg;
    }
    void process_img(const sensor_msgs::msg::Image::SharedPtr msg)
    {
	//processing recieved image

	unique_ptr<PointCloud> cloud_msg  = std::make_unique<PointCloud>();
	cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
	if (cv_ptr == nullptr) {
		RCLCPP_ERROR(this->get_logger(), "Failed to convert the image");
		return;
	}
	gray_image = cv_ptr->image;
	// cv::cvtColor(cv_ptr->image, gray_image, cv::COLOR_RGB2GRAY);

	
	
	// vector<rclcpp::Parameter> all_new_parameters{rclcpp::Parameter("binary_threshold_low",100),rclcpp::Parameter("binary_threshold_high",240)};
	// this->set_parameters(all_new_parameters);
	// // cout<<binary_threshold_low<<" "<<binary_threshold_high<<endl;
	// cv::inRange(gray_image, cv::Scalar(binary_threshold_low), cv::Scalar(binary_threshold_high), gray_image); 
	
	sensor_msgs::msg::Image::SharedPtr gray_msg_ros = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", gray_image).toImageMsg();
	publisher_forward->publish(*gray_msg_ros);
	// cv::imshow("window",gray_image);
	// cv::waitKey(1);

	cv::Mat nonZeroCoordinates;
	cv::findNonZero(gray_image, nonZeroCoordinates);

	
	
	cv::Mat downsampled;

	downsample(nonZeroCoordinates,downsampled);

	nonZeroCoordinates = downsampled;

	vector<pair<int,int>> points;

	for (int i = 0; i < nonZeroCoordinates.rows; ++i) {
        cv::Point point = nonZeroCoordinates.at<cv::Point>(i);
		std::pair<int,int> p = process_point(point.y,point.x);
        if (p.first != 0 and p.second!=0)
			points.emplace_back(p);
    }
	
	

	// Now `downsampled` contains the grid-based downsampled points

		


	// //some calculation s
	// float roll = 0;
	// float pitch = 0;//-22* M_PI / 180;
	// float yaw = 0;
	// float h = 0.8; //1.373;
	// int m = 3;
	// int n = 3;
	// vector<double> k(9), nor(3), uv(3);

	// double cy, cr, sy, sr, sp, cp;
	// cy = cos(yaw);
	// sy = sin(yaw);
	// cp = cos(pitch);
	// sp = sin(pitch);
	// cr = cos(roll);
	// sr = sin(roll);
	// k[0] = cr*cy+sp*sr+sy;
	// k[1] = cr*sp*sy-cy*sr;
	// k[2] = -cp*sy;
	// k[3] = cp*sr;
	// k[4] = cp*cr;
	// k[5] = sp;
	// k[6] = cr*sy-cy*sp*sr;
	// k[7] = -cr*cy*sp -sr*sy;
	// k[8] = cp*cy;

	// nor[0] = 0;
	// nor[1] = 1.0;
	// nor[2] = 0;

	// // //what does this do?
	// matmul(k.data(), nor.data(), uv.data());

	// // no of points to map
	// cv::Size s = nonZeroCoordinates.size();
	// int rows = s.height;
	// // std::cout << "rows : " << rows << '\n';
	// auto caminfo = this->camera_info.k;
	// Eigen::Map<Matrix<double,3,3,RowMajor> > mat(caminfo.data());
	// mat = mat.inverse();
	// double *inv_caminfo = mat.data();

	// vector<double> kin_uv(3*rows), uv_hom(3*rows), denom(rows);


	// //device
	// double *d_uv_hom, *d_kin_uv, *d_caminfo, *d_denom, *d_uv;
	// auto result = cudaMalloc((void **) &d_uv_hom, sizeof(double)*3*rows);
	// if(result != cudaSuccess){
	// 	cerr << "uv home failed!\n";
	// }
	// result = cudaMalloc((void **) &d_kin_uv, sizeof(double)*3*rows);
	// 	if(result != cudaSuccess){
	// 	cerr << "kin uv  failed!\n";
	// }
	// result = cudaMalloc((void **) &d_caminfo, sizeof(double)*9);
	// 	if(result != cudaSuccess){
	// 	cerr << "cam info failed!\n";
	// }

	// result = cudaMalloc((void **) &d_denom, sizeof(double)*rows);
	// 	if(result != cudaSuccess){
	// 	cerr << "denom failed!\n";
	// }
	// result = cudaMalloc((void **) &d_uv, sizeof(double)*3);
	// 	if(result != cudaSuccess){
	// 	cerr << "uv  failed!\n";
	// }
 
	// //gathering data for all points
	// if(rows == 0){
	// 	//if no points found, will publish the previous non zero pointcloud
	// 	std::cerr << "no points to project!!!";
	// 	publisher_->publish(pub_pointcloud);
	// }else{
	// for (int i = 0; i < rows; i++)
	//  {
	//      int x = nonZeroCoordinates.at<cv::Point>(i).x;
	//      int y = nonZeroCoordinates.at<cv::Point>(i).y;
	//      uv_hom[i*3] = x;
	//      uv_hom[i*3+1] = y; 
	//      uv_hom[i*3+2] = 1;
	//  }

	 
	// //copying to device
	// cudaMemcpy(d_caminfo, inv_caminfo, sizeof(double)*9, cudaMemcpyHostToDevice);
	// cudaMemcpy(d_uv_hom, uv_hom.data(), sizeof(double)*3*rows, cudaMemcpyHostToDevice);
	// cudaMemcpy(d_uv, uv.data(), sizeof(double)*3, cudaMemcpyHostToDevice);
	
	// //batch multiplication
	// //launching rows no of threads and one block
	// dev_matmul<<<BLOCKS, (rows+BLOCKS-1)/BLOCKS>>>(d_caminfo, d_uv_hom, d_kin_uv, rows);
	// dot<<<BLOCKS, (rows+BLOCKS-1)/BLOCKS>>>(d_uv, d_kin_uv, d_denom, rows);
	
	// cudaMemcpy(kin_uv.data(), d_kin_uv, sizeof(double)*3*rows, cudaMemcpyDeviceToHost);
	// cudaMemcpy(denom.data(), d_denom, sizeof(double)*rows, cudaMemcpyDeviceToHost);

	// Lane Following -----------------------------------------------------------------------------------------------------
	cv::Mat ipm_image = cv::Mat::zeros(gray_image.size(), gray_image.type());
	
	std::vector<cv::Point> filtered_white_pixel_indices;
	
	// cout<<denom[0]<<endl;;
	for(int i=0; i < points.size(); ++i) {
		pcl::PointXYZ vec;
		//fix, make it work im not doing it
		

		vec.x = points[i].first;
		vec.y = - points[i].second;
		vec.z =  0 ;//h * kin_uv[i*3+1]/denom[i];
		// cout<<vec.x<<vec.y<<vec.z<<endl;
		cloud_msg->points.push_back(vec);
		vec.y *= 100;
		vec.x*= 100;
		vec.y += ipm_image.cols/2;
		vec.x = ipm_image.rows - vec.x;
		vec.y = ipm_image.cols - vec.y;
		
		
		
		// cout<<vec.y<<" "<<ipm_image.cols<<" "<<vec.x<<" "<<ipm_image.rows<<endl;
		// if (vec.y<ipm_image.cols and vec.x < ipm_image.rows) ipm_image.at<u_char>(vec.y,vec.x) = 255;
		if (vec.y >= 0 && vec.y < ipm_image.rows && vec.x >= 0 && vec.x < ipm_image.cols) ipm_image.at<u_char>(vec.x,vec.y) = 255;
		// cv::Point point;
		// point.x = vec.x;
		// point.y = vec.y;

		// filtered_white_pixel_indices.push_back(point);

		
	}
	
	cv::imshow("window",ipm_image);
	cv::waitKey(1);
	
	
	
	//---------------------------------------------------------------------------------------------------------------------------------
	// cudaFree(d_uv_hom);
	
	// cudaFree(d_uv);
	// cudaFree(d_kin_uv);
	// cudaFree(d_caminfo);
	// cudaFree(d_denom);   
	cloud_msg->height   = 1;
	cloud_msg->width    = cloud_msg->points.size();
	cloud_msg->is_dense = false;
	pcl::toROSMsg(*cloud_msg, pub_pointcloud);
	pub_pointcloud.header.frame_id =frame;
	pub_pointcloud.header.stamp = rclcpp::Clock().now();


	sensor_msgs::msg::Image::SharedPtr ipm_msg_ros = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", ipm_image).toImageMsg();

	publisher_image->publish(*ipm_msg_ros);

	 // Publishing our cloud image
	publisher_->publish(pub_pointcloud);

	try {
        // Lookup the transform from base_link to map
        geometry_msgs::msg::TransformStamped transform_stamped = tf_buffer_.lookupTransform("map", "base_link", tf2::TimePointZero);

        // Transform the point cloud
        for (auto& point : cloud_msg->points) {
            geometry_msgs::msg::PointStamped point_in, point_out;
            point_in.header.frame_id = "base_link";
            point_in.point.x = point.x;
            point_in.point.y = point.y;
            point_in.point.z = point.z;

            tf2::doTransform(point_in, point_out, transform_stamped);

            // Update the point cloud with transformed points
            point.x = point_out.point.x;
            point.y = point_out.point.y;
            point.z = point_out.point.z;
        }
    } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "Could NOT transform: %s", ex.what());
        return;
    }

    // Continue with publishing the transformed point cloud
	
	pcl::toROSMsg(*cloud_msg, pub_pointcloud);
	pub_pointcloud.header.frame_id ="map";
	pub_pointcloud.header.stamp = rclcpp::Clock().now();
    // cloud_msg->header.frame_id = "map"; // Set the frame_id to map after transformation
    publisher_map->publish(pub_pointcloud);

	cloud_msg->points.clear();
	 
	}

	std::pair<int,int> process_point(int y, int x) {
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
    Eigen::Map<Matrix<double,3,3,RowMajor>> kin(caminfo.data());
    kin = kin.inverse().eval();

    // Convert the pixel coordinates (x, y) to homogeneous coordinates
    Eigen::Vector3d uv_hom(x, y, 1);

    // Map pixel coordinates to 3D camera ray
    Eigen::Vector3d kin_uv = kin * uv_hom;

    // Calculate the denominator for scaling (distance along the ray to the plane)
    double denom = kin_uv.dot(nc);

    // Ensure denom is not zero to avoid division by zero
    if (denom != 0) {
        // Scale the ray by the height of the plane h
        pcl::PointXYZ point;
        point.x = h * kin_uv[2] / denom;
        point.y = -h * kin_uv[0] / denom;
        point.z = 0; // z-coordinate is zero on the ground plane
		std::pair<int,int> p;
		p.first = point.x;
		p.second = point.y;
		return p;
        
    } else {
        std::cerr << "Denominator is zero, invalid projection for point (" << x << ", " << y << ")" << std::endl;
		std::pair<int,int> p;
		p.first = 0;
		p.second = 0;
		return p;
    }
	
    
}
    


	

	// rcl_interfaces::msg::SetParametersResult parameter_callback(
    // const std::vector<rclcpp::Parameter> & params)
	// {
	// 	rcl_interfaces::msg::SetParametersResult result;
	// 	result.successful = true;
		
	// 	for (const auto & param : params) {
	// 	if (param.get_name() == "masked_image_topic" && param.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
	// 		masked_image_topic = param.as_string();
			
	// 		// Destroy the old subscription
	// 		subscription_img.reset();
			
	// 		// Create a new subscription with the updated topic
	// 		subscription_img = this->create_subscription<sensor_msgs::msg::Image>(masked_image_topic, 10, std::bind(&IPM::process_img, this, _1));
			
	// 		// rclcpp::RCLCPP_INFO(this->get_logger(), "Changed subscription topic to: %s", subscription_topic_.c_str());
	// 	}
	// 	}
		
	// 	return result;
	// }
    private:
		rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr subscription_caminfo;
		rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_img;
		rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
		rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_map;

		rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_forward;
		rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_image;
		cv::Mat gray_image;
		sensor_msgs::msg::CameraInfo camera_info;
		typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
		string masked_image_topic;
		string camera_topic;
		OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
		int binary_threshold_low;
		int binary_threshold_high;
		float ipm_pitch;
		float ipm_height;
		string frame;
		sensor_msgs::msg::PointCloud2 pub_pointcloud;

		tf2_ros::Buffer tf_buffer_;
		tf2_ros::TransformListener tf_listener_;

};



int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IPM>());
  rclcpp::shutdown();
  return 0;
}
