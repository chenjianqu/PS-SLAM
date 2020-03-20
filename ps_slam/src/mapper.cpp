#include <stdio.h>
#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <mutex>
#include <list>
#include <thread>

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
//#include <pcl/visualization/pcl_visualizer.h>
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
	void Mapping();//建图主函数
	void SavePointCloud();
	void SaveTrackAndImg(const cv::Mat imgColor,const cv::Mat imgDepth,Eigen::Isometry3d &Tcw);
	
	void InsertFrame(cv::Mat color,cv::Mat depth,Eigen::Isometry3d Tcw);
	void GetFrame(cv::Mat* color,cv::Mat* depth,Eigen::Isometry3d *Tcw);
	void PopFrame();
	int GetFrameListSize();
	
	void SetRun(bool isRun_);
	bool GetRun();
protected:
	void VoxelFilter();
	
	PointCloud::Ptr pointCloud;
	
	//相机内参
	float fx;
    float fy;
    float cx;
    float cy;
	float depthFactor;
	
	//计数器
	unsigned long counter;
	
	//运行状态
	bool isRun;
	
	std::mutex frameUpdateMutex;
	std::mutex RunMutex;
	
	//等待队列
	std::list<cv::Mat> colorList;
	std::list<cv::Mat> depthList;
	std::list<Eigen::Isometry3d> poseList;
	
	//pcl::visualization::CloudViewer viewer;
};


class CallBackClass
{
public:
	CallBackClass(const string &strSettingsFile);
	void CallBack(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);
	
	tf::TransformListener* listener;
	
	Mapper *mapper;
	std::thread* mapperThread;

protected:
	unsigned long frameCounter;
};




int main(int argc, char** argv)
{
	ros::init(argc, argv, "mapper_node");//初始化节点
	ros::start();//启动节点
	
	if(argc != 2)
    {
        cout<<"需要传入参数：配置文件路径" << endl;
        ros::shutdown();//关闭节点
        return 1;
    }
	
	ROS_INFO_STREAM("Initing");
	
	ros::NodeHandle nh;
	
	CallBackClass cb(argv[1]);//初始化

	//接受RGB图和深度图
	message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/ps_node", 10);
	message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/orbslam2/depth", 10);
	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
	message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub,depth_sub);
	sync.registerCallback(boost::bind(&CallBackClass::CallBack,&cb,_1,_2));//_1表示消息1，_2表示消息2
	
	ROS_INFO_STREAM("Mapper Inited");
	
	ros::spin();
	
	cb.mapper->SetRun(false);//关闭线程
	
	ros::Duration(2).sleep();
	
	ROS_INFO_STREAM("Mapper节点结束");
		
	return 0;
}



CallBackClass::CallBackClass(const string &strSettingsFile)
{
	frameCounter=0;
	listener=new tf::TransformListener();//tf监听器
	mapper=new Mapper(strSettingsFile);
	mapperThread = new thread(&Mapper::Mapping, mapper);//线程启动
}




void CallBackClass::CallBack(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD)
{
	
	//获取相机位姿
	Eigen::Isometry3d Tcw=Eigen::Isometry3d::Identity();
	ros::Time timestamp= msgRGB->header.stamp;
	tf::StampedTransform m;
	listener->lookupTransform("world", "orb_slam2", timestamp, m);//查找坐标转换
	tf::transformTFToEigen(m, Tcw);
	
	ROS_INFO_STREAM(frameCounter);
	frameCounter++;
	//cout<<"counter:"<<counter<<endl;
	//cout<<"RGB Time:"<<msgRGB->header.stamp<<endl;
	//cout<<"Depth Time:"<<msgD->header.stamp<<endl;
	//cout<<Tcw.matrix()<<endl<<endl;
	
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
	cv::Mat color=cv_ptrRGB->image.clone();
	cv::Mat depth=cv_ptrD->image.clone();
	
	//插入关键帧
	mapper->InsertFrame(color,depth,Tcw);
}






Mapper::Mapper(const string &strSettingsFile):isRun(true)
{
	//清空pose.txt文件
	ofstream f("./data/pose.txt",ios::trunc);
	f.close();
	
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

	PointCloud::Ptr pointCloudPtr( new PointCloud );
	//pointCloud=boost::shared_ptr<PointCloud>();
	pointCloud=pointCloudPtr;
	
	counter=0;
	
	//pcl::visualization::CloudViewer viewer_("pcd viewer");
	//viewer=viewer_;
}

//设置运行状态
void Mapper::SetRun(bool isRun_)
{
	unique_lock<mutex> lock(RunMutex);
	isRun=isRun_;
}

//获取运行状态
bool Mapper::GetRun()
{
	unique_lock<mutex> lock(RunMutex);
	return isRun;
}

//在消息队列的尾部插入帧
void Mapper::InsertFrame(cv::Mat color,cv::Mat depth,Eigen::Isometry3d Tcw)
{
	unique_lock<mutex> lock(frameUpdateMutex);
	colorList.push_back(color);
	depthList.push_back(depth);
	poseList.push_back(Tcw);
}


//获取消息队列中的第一帧
void Mapper::GetFrame(cv::Mat* color,cv::Mat* depth,Eigen::Isometry3d *Tcw)
{
	unique_lock<mutex> lock(frameUpdateMutex);	
	color=&(colorList.front());//获得第一个元素
	depth=&(depthList.front());
	Tcw=&(poseList.front());
}

//删除消息队列中的第一帧
void Mapper::PopFrame()
{
	unique_lock<mutex> lock(frameUpdateMutex);
	colorList.pop_front();//删除第一个元素
	depthList.pop_front();
	poseList.pop_front();
}

//获取消息队列大小
int Mapper::GetFrameListSize()
{
	unique_lock<mutex> lock(frameUpdateMutex);
	return colorList.size();
}



void Mapper::VoxelFilter()
{
	cout<<"对全局点云进行体素滤波..."<<endl;
	pointCloud->is_dense = false;
	
	//体素滤波器（Voxel Filter）进行降采样
    pcl::VoxelGrid<PointT> voxel_filter; 
    voxel_filter.setLeafSize( 0.01, 0.01, 0.01 );       //分辨率
    PointCloud::Ptr tmp ( new PointCloud );
    voxel_filter.setInputCloud( pointCloud );
    voxel_filter.filter( *tmp );
    tmp->swap( *pointCloud );
    
    cout<<"点云剩余数量"<<pointCloud->size()<<endl<<endl;
}



void Mapper::SavePointCloud()
{
    pcl::io::savePCDFileBinary("map.pcd", *pointCloud );
}


//保存位姿和图片
void Mapper::SaveTrackAndImg(const cv::Mat imgColor,const cv::Mat imgDepth,Eigen::Isometry3d &Tcw)
{
	cv::imwrite("./data/color/"+to_string(counter)+".png",imgColor);
	cv::imwrite("./data/depth/"+to_string(counter)+".exr",imgDepth);//这里的深度图为CV_32F格式，故保存为*.exr
	
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



void Mapper::Mapping()
{
	while(GetRun())
	{
		int size=GetFrameListSize();
		if(size>0)
		{
			cout<<"消息队列长度："<<size<<endl;
			//获取帧和位姿
			cv::Mat color;
			cv::Mat depth;
			Eigen::Isometry3d Tcw;
			//获取帧数据
			{
				unique_lock<mutex> lock(frameUpdateMutex);	
				color=colorList.front();//获得第一个元素
				depth=depthList.front();
				Tcw=poseList.front();
			}
			
			Eigen::Isometry3d Twc=Tcw.inverse();//求逆变换
			PointCloud::Ptr current( new PointCloud );//当前帧的点云

			for ( int v=0; v<color.rows; v++ )
			{
				for ( int u=0; u<color.cols; u++ )
				{
					float d = depth.ptr<float> (v)[u]; // 深度值
					if ( d==0 ) continue; // 为0表示没有测量到
					if ( d >= 4.0 ) continue; // 深度太大时不稳定，去掉
					Eigen::Vector3d point; 
					point[2] = double(d)/depthFactor; 
					point[0] = (u-cx)*point[2]/fx;
					point[1] = (v-cy)*point[2]/fy; 
					//设点P,在相机坐标为Pc，世界坐标为Pw，该相机位姿为:Tcw，那么有：Pc=Tcw*Pw，Pw=Twc*Pc
					Eigen::Vector3d pointWorld = Twc*point;
					PointT p ;
					p.x = pointWorld[0];
					p.y = pointWorld[1];
					p.z = pointWorld[2];
					p.b = color.data[ v*color.step+u*color.channels() ];
					p.g = color.data[ v*color.step+u*color.channels()+1 ];
					p.r = color.data[ v*color.step+u*color.channels()+2 ];
					current->points.push_back(p);
				}
			}

			//利用统计滤波器方法去除孤立点。
			PointCloud::Ptr tmp ( new PointCloud );
			pcl::StatisticalOutlierRemoval<PointT> statistical_filter;
			statistical_filter.setMeanK(50);
			statistical_filter.setStddevMulThresh(1.0);
			statistical_filter.setInputCloud(current);
			statistical_filter.filter( *tmp );

			(*pointCloud) += *tmp;
			
			//viewer.showCloud(pointCloud);

			//viewer.showCloud(pointCloud);
			cout<<"当前点云数量："<<pointCloud->size()<<endl<<endl;	
			
			counter++;
			PopFrame();//从消息队列中删除该帧
			
			//每隔20帧进行一次体素滤波，减少地图点数量
			if(counter%10==0)
			{
				VoxelFilter();
			}
			
		}
	}
	
	VoxelFilter();
	SavePointCloud();
	cout<<"最终全局点云共有"<<pointCloud->size()<<"个点."<<endl;
	cout<<"建图线程结束"<<endl;
}
