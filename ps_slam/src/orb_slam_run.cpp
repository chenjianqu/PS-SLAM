#include <stdio.h>
#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <std_msgs/Time.h>

#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>

#include"System.h"


using namespace std;



class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM);
    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);
	void SetPublisher(image_transport::Publisher* pub_rgb_,image_transport::Publisher* pub_depth_);
protected:
    ORB_SLAM2::System* mpSLAM;
	tf::TransformBroadcaster* br;
	image_transport::Publisher* pub_rgb;
	image_transport::Publisher* pub_depth;
	unsigned long counter;
	
	void MatToTransform(cv::Mat &Tcw,tf::Transform &m);
	
};


int main(int argc, char** argv)
{
	ros::init(argc, argv, "orb_slam_node");//初始化节点
	ros::start();//启动节点
	if(argc != 3)
    {
        cout<<"需要传入参数：视觉词典路径 配置文件路径" << endl;
        ros::shutdown();//关闭节点
        return 1;
    }
	
	//初始化ORB-SLAM2
	ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::RGBD,true);
	ImageGrabber igb(&SLAM);
	
	ros::NodeHandle nh;

	//接受RGB图和深度图
    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/rgb/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/camera/depth_registered/image_raw", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub,depth_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD,&igb,_1,_2));

	image_transport::ImageTransport it(nh);
	image_transport::Publisher pub_rgb = it.advertise("/orbslam2/rgb", 1);
	image_transport::Publisher pub_depth = it.advertise("/orbslam2/depth", 1);
	igb.SetPublisher(&pub_rgb,&pub_depth);
	
	
	ros::spin();
	SLAM.Shutdown();
	SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
	ros::shutdown();
	return 0;
}


ImageGrabber::ImageGrabber(ORB_SLAM2::System* pSLAM):
	mpSLAM(pSLAM),
	counter(0)
{
	br=new tf::TransformBroadcaster();
}

void ImageGrabber::SetPublisher(image_transport::Publisher* pub_rgb_,image_transport::Publisher* pub_depth_)
{
	pub_rgb=pub_rgb_;
	pub_depth=pub_depth_;
}

void ImageGrabber::MatToTransform(cv::Mat &Tcw,tf::Transform &m)
{
	//设置平移
	m.setOrigin(
		tf::Vector3(
			Tcw.at<float>(0,3),
			Tcw.at<float>(1,3),
			Tcw.at<float>(2,3)
		)
	);
	
	//设置旋转
	tf::Matrix3x3 Rcw;
	Rcw.setValue( //Mat转换为Matrix
		Tcw.at<float>(0,0),Tcw.at<float>(0,1),Tcw.at<float>(0,2),  
		Tcw.at<float>(1,0),Tcw.at<float>(1,1),Tcw.at<float>(1,2),
		Tcw.at<float>(2,0),Tcw.at<float>(2,1),Tcw.at<float>(2,2)
	);
	
	tf::Quaternion q;
	Rcw.getRotation(q);
	m.setRotation(q);
}


void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD)
{
	
	ros::Time timestamp= msgRGB->header.stamp;
	
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try{
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
	}
    catch (cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try{
        cv_ptrD = cv_bridge::toCvShare(msgD);
	}
    catch (cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
	
	//调用ORB-SLAM2
	cv::Mat Tcw=mpSLAM->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec());
	
	//如果不是关键帧,就退出吧
	if(!mpSLAM->GetIsKeyFrame())
		return;
	
	tf::Transform m;
	MatToTransform(Tcw,m);
	
	//发布rgb和深度图
	pub_rgb->publish(msgRGB);
	pub_depth->publish(msgD);
	
	//cout<<"RGB Time:"<<msgRGB->header.stamp<<endl;
	//cout<<"Depth Time:"<<msgD->header.stamp<<endl;
	//cout<<Tcw<<endl<<endl;
	
	//发布坐标
	br->sendTransform(tf::StampedTransform(m, timestamp, "world", "orb_slam2"));
	
	counter++;
	cout<<"发布关键帧序号："<<counter<<endl;
}

