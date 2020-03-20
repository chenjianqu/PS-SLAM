#include <stdio.h>
#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


#include <pcl/point_types.h> 
#include <pcl/io/pcd_io.h> 
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
//#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/statistical_outlier_removal.h>


// 定义点云使用的格式：这里用的是XYZRGB
typedef pcl::PointXYZRGB PointT; 
typedef pcl::PointCloud<PointT> PointCloud;

using namespace std;


class Mapper
{
public:
    Mapper(const string &strSettingsFile);
	void GrabMsg(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);
	void CallbackTest(const sensor_msgs::ImageConstPtr& msg);
	void TransformToEigen(tf::Transform &m,Eigen::Isometry3d &Tcw);
	void Mapping(const cv::Mat imgColor,const cv::Mat imgDepth,Eigen::Isometry3d &Tcw);
	void SavePointCloud();
	void SaveTrackAndImg(const cv::Mat imgColor,const cv::Mat imgDepth,Eigen::Isometry3d &Tcw);

protected:
	tf::TransformListener* listener;
	PointCloud::Ptr pointCloud;
	//pcl::visualization::CloudViewer viewer;
	
	//相机内参
	float fx;
    float fy;
    float cx;
    float cy;
	float depthFactor;
	
	//计数器
	unsigned long counter;
};


int main(int argc, char** argv)
{
	ros::init(argc, argv, "mapper_test_node");//初始化节点
	ros::start();//启动节点
	
	if(argc != 2)
    {
        cout<<"需要传入参数：配置文件路径" << endl;
        ros::shutdown();//关闭节点
        return 1;
    }
	
	ROS_INFO_STREAM("Initing");
	
	ros::NodeHandle nh;
	
	Mapper mapper(argv[1]);

	
	//接受RGB图和深度图
	message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/rgb/image_raw", 10);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/camera/depth_registered/image_raw", 10);
	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
	message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub,depth_sub);
	sync.registerCallback(boost::bind(&Mapper::GrabMsg,&mapper,_1,_2));//最后两个参数是传入两个回调函数的参数
	
	//image_transport::ImageTransport it(nh);
 	//image_transport::Subscriber sub = it.subscribe( "/ps_node", 1, boost::bind(&Mapper::CallbackTest,&mapper,_1) );
	ROS_INFO_STREAM("Mapper Inited");
	
	ros::spin();
	
	//string imgPath="/media/chen/chen/SLAM/projects_test/PointCloudMap/data/";
	//string imgPath="/media/chen/chen/Robot/slam_ws/data/";
	//cv::Mat color=cv::imread(imgPath+"color/1.png");
	//cv::Mat depth=cv::imread(imgPath+"depth/1.pgm",-1);
	//Eigen::Isometry3d Tcw=Eigen::Isometry3d::Identity();
	//mapper.Mapping(color,depth,Tcw);
	mapper.SavePointCloud();
	
	ROS_INFO_STREAM("Mapper Ending");
		
	return 0;
}


Mapper::Mapper(const string &strSettingsFile)
{
	cv::FileStorage fSettings(strSettingsFile, cv::FileStorage::READ);
    if(!fSettings.isOpened()){
       cerr << "无法打开配置文件: " << strSettingsFile << endl;
       exit(-1);
    }
	//设置相机内参
	
    fx = fSettings["Camera.fx"];
    fy = fSettings["Camera.fy"];
    cx = fSettings["Camera.cx"];
    cy = fSettings["Camera.cy"];
	depthFactor=fSettings["Camera.factor"];
/*
	fx = 518.0;
    fy = 519.0;
    cx = 325.5;
    cy = 253.5;
	depthFactor=1000.0;
*/
	listener=new tf::TransformListener();//tf监听器
	
	PointCloud::Ptr pointCloudPtr( new PointCloud );
	pointCloud=pointCloudPtr;
	
	//pcl::visualization::CloudViewer viewer_("pcd viewer");
	//viewer=viewer_;
	
	counter=0;
	
	ROS_INFO_STREAM("Parameters：");
	cout<<"fx："<<fx<<endl;
	cout<<"fy："<<fy<<endl;
	cout<<"cx："<<cx<<endl;
	cout<<"cy："<<cy<<endl;
	cout<<"depthFactor："<<depthFactor<<endl;
	
}


void Mapper::TransformToEigen(tf::Transform &m,Eigen::Isometry3d &Tcw)
{
	Eigen::Translation3f t_cw(m.getOrigin().getX(),m.getOrigin().getY(), m.getOrigin().getZ());
    double roll, pitch, yaw;
    tf::Matrix3x3(m.getRotation()).getEulerYPR(yaw, pitch, roll);
    Eigen::AngleAxisf rot_x(roll, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf rot_y(pitch, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf rot_z(yaw, Eigen::Vector3f::UnitZ());
    //Tcw = (t_cw * rot_z * rot_y * rot_x).matrix();
	
}



void Mapper::CallbackTest(const sensor_msgs::ImageConstPtr& msg)
{
	cout<<"RGB Time:"<<msg->header.stamp<<endl;
}


//回调函数
void Mapper::GrabMsg(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD)
{
	counter++;
	if(counter!=10)
		return;
	
	//获取相机位姿
	
	cout<<"counter:"<<counter<<endl;
	cout<<"RGB Time:"<<msgRGB->header.stamp<<endl;
	cout<<"Depth Time:"<<msgD->header.stamp<<endl;
	
	
	vector<int> d_compression_param;
    d_compression_param.push_back(CV_IMWRITE_PNG_COMPRESSION);
    d_compression_param.push_back(0);// png Highest quality
	cv_bridge::CvImagePtr image_depth = cv_bridge::toCvCopy(msgD);
	cv::imwrite("./data/depth/"+to_string(counter)+".png",image_depth->image,d_compression_param);
	
	
	
	//获取图像
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
	Eigen::Isometry3d Tcw=Eigen::Isometry3d::Identity();
	
	
	
	
	Mapping(cv_ptrRGB->image,cv_ptrD->image,Tcw);	
	//SaveTrackAndImg(cv_ptrRGB->image,cv_ptrD->image,Tcw);//位姿和图片保存
}


void Mapper::SavePointCloud()
{
	pointCloud->is_dense = false;
	
	//体素滤波器（Voxel Filter）进行降采样
    pcl::VoxelGrid<PointT> voxel_filter; 
    voxel_filter.setLeafSize( 0.01, 0.01, 0.01 );       //分辨率
    PointCloud::Ptr tmp ( new PointCloud );
    voxel_filter.setInputCloud( pointCloud );
    voxel_filter.filter( *tmp );
    //tmp->swap( *pointCloud );
    
    cout<<"滤波之后，点云共有"<<pointCloud->size()<<"个点."<<endl;
	
    pcl::io::savePCDFileBinary("map.pcd", *pointCloud );
}


//保存位姿和图片
void Mapper::SaveTrackAndImg(const cv::Mat imgColor,const cv::Mat imgDepth,Eigen::Isometry3d &Tcw)
{
	cout<<"imgColor.type()="<<imgColor.type()<<endl;
	cout<<"imgDepth.type()="<<imgDepth.type()<<endl;
	
	 vector<int> d_compression_param;
     d_compression_param.push_back(CV_IMWRITE_PNG_COMPRESSION);
     d_compression_param.push_back(0);// png Highest quality

	cv::imwrite("./data/color/"+to_string(counter)+".png",imgColor,d_compression_param);
	cv::imwrite("./data/depth/"+to_string(counter)+".exr",imgDepth);
	
	Eigen::Matrix4d TcwM=Tcw.matrix();
	Eigen::Matrix3d Rcw;
	Rcw<<TcwM(0,0),TcwM(0,1),TcwM(0,2),
		 TcwM(1,0),TcwM(1,1),TcwM(1,2),
		 TcwM(2,0),TcwM(2,1),TcwM(2,2);
	Eigen::Quaterniond q(Rcw);
	
	Eigen::Vector3d t=Eigen::Vector3d(TcwM(0,3),TcwM(1,3),TcwM(2,3));

	std::ofstream poseStream;
	poseStream.open("./data/pose.txt",std::ios::app);
	poseStream <<t(0,0)<<t(1,0)<<t(2,0)<<q.x()<<q.y()<<q.z()<<q.w()<< endl;
	poseStream.close();
}


void Mapper::Mapping(const cv::Mat imgColor,const cv::Mat imgDepth,Eigen::Isometry3d &Tcw)
{
	Eigen::Isometry3d Twc=Tcw.inverse();//求逆变换
	//Eigen::Isometry3d Twc=Tcw;
	
	PointCloud::Ptr current( new PointCloud );
	
	cout<<"imgColor type:"<<imgColor.type()<<endl;
	cout<<"imgDepth type:"<<imgDepth.type()<<endl;
	
	cout<<"构建该帧点云"<<endl;
	
	for ( int v=0; v<imgColor.rows; v++ ){
		for ( int u=0; u<imgColor.cols; u++ ){
			
			float d = imgDepth.ptr<float> ( v )[u]; // 深度值
			if(v==u)
				cout<<d<<" ";
			if ( d<0.0001 ) continue; // 为0表示没有测量到
			//if ( d >= 35000 ) continue; // 深度太大时不稳定，去掉
			Eigen::Vector3d point; 
			point[2] = d/depthFactor; 
			point[0] = (u-cx)*point[2]/fx;
			point[1] = (v-cy)*point[2]/fy; 
			//Eigen::Vector3d pointWorld = Twc*point;
			Eigen::Vector3d pointWorld=point;

			PointT p ;
			p.x = pointWorld[0];
			p.y = pointWorld[1];
			p.z = pointWorld[2];
			p.b = imgColor.data[ v*imgColor.step+u*imgColor.channels() ];
			p.g = imgColor.data[ v*imgColor.step+u*imgColor.channels()+1 ];
			p.r = imgColor.data[ v*imgColor.step+u*imgColor.channels()+2 ];
			current->points.push_back( p );
		}
	}
	
	cout<<"对该帧点云进行统计滤波"<<endl;

	//利用统计滤波器方法去除孤立点。
	PointCloud::Ptr tmp ( new PointCloud );
	pcl::StatisticalOutlierRemoval<PointT> statistical_filter;
	statistical_filter.setMeanK(50);
	statistical_filter.setStddevMulThresh(1.0);
	statistical_filter.setInputCloud(current);
	statistical_filter.filter( *tmp );
	
	cout<<"将该帧点云加入大点云"<<endl;
	
	(*pointCloud) += *tmp;

	//viewer.showCloud(pointCloud);
	cout<<"点云共有"<<pointCloud->size()<<"个点."<<endl;
	
	//viewer.showCloud(pointCloud);
	//getchar();
}
