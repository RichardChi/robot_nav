#include <ros/ros.h>
#include <math.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>

#include <tf/transform_broadcaster.h>

#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <octomap/octomap.h>
#include <octomap_ros/conversions.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/Octomap.h>

const double PI = 3.1415926;

pcl::PointCloud<pcl::PointXYZ>::Ptr front_pointcloud(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr all_pointcloud(new pcl::PointCloud<pcl::PointXYZ>());

geometry_msgs::PoseStamped uavPose;
double timeLaserCloudFront;
double timeOdometry;
double timeMap;
bool newLaserCloudFront;
bool newOdomtry;
bool firstFlag = true;
double pose_Translation[7];
double timeCloud;

void laserCloudCallback(const sensor_msgs::PointCloud2ConstPtr&  rawPointCloud)
{
	timeLaserCloudFront = rawPointCloud->header.stamp.toSec();
	front_pointcloud->clear();
	pcl::fromROSMsg(*rawPointCloud, *front_pointcloud);
	std::vector<int> indices;
   	pcl::removeNaNFromPointCloud(*front_pointcloud,*front_pointcloud, indices);
   	indices.clear();

	pcl::VoxelGrid<pcl::PointXYZ> sor;
  	sor.setInputCloud (front_pointcloud);
  	sor.setLeafSize (0.04f, 0.04f, 0.04f);
  	sor.filter (*front_pointcloud);

  	Eigen::Affine3d tran = Eigen::Affine3d::Identity();
  	tran.translation()<<0.0, 0.0, 0.0;
  	tran.rotate (Eigen::AngleAxisd (0.5 * PI, Eigen::Vector3d::UnitY())
  		        * Eigen::AngleAxisd (-0.5 * PI, Eigen::Vector3d::UnitZ()));
  	pcl::transformPointCloud(*front_pointcloud, *front_pointcloud, tran);

	newLaserCloudFront = true;
}

void odometryCallback(const nav_msgs::Odometry::ConstPtr& rawOdometry)
{
	timeOdometry = rawOdometry->header.stamp.toSec();
	//uavPose = rawOdometry->pose;
	pose_Translation[0] =  rawOdometry->pose.pose.position.x;
	pose_Translation[1] =  rawOdometry->pose.pose.position.y;
	pose_Translation[2] =  rawOdometry->pose.pose.position.z;
	pose_Translation[3] =  rawOdometry->pose.pose.orientation.x;
	pose_Translation[4] =  rawOdometry->pose.pose.orientation.y;
	pose_Translation[5] =  rawOdometry->pose.pose.orientation.z;
	pose_Translation[6] =  rawOdometry->pose.pose.orientation.w;

	newOdomtry = true;
}

int main( int argc, char** argv )
{
	ros::init(argc, argv, "data_pretreatment");
	ROS_INFO("data pretreatment test");
	ros::NodeHandle nh;
	ros::Subscriber subLaserCloudFullRes = nh.subscribe<sensor_msgs::PointCloud2>("/points", 2, laserCloudCallback);
	ros::Subscriber subLaserOdometry = nh.subscribe<nav_msgs::Odometry>("/odometry", 5, odometryCallback);
	ros::Publisher pubLaserCloud = nh.advertise<sensor_msgs::PointCloud2>("/pcs", 2);
	ros::Publisher pubLaserCloudFull = nh.advertise<sensor_msgs::PointCloud2>("/full_pcs", 2);


	tf::TransformBroadcaster tfBroadcaster;
	tf::Transform transforms;
   	tf::StampedTransform tf_body;
   	/*tf_body.frame_id_ = "/map";
   	tf_body.child_frame_id_ = "/body";
   	tf_body.stamp_ = ros::Time::now();
   	tf_body.setOrigin(tf::Vector3(0,0,0));
	tf_body.setRotation(tf::Quaternion(0,0,0,1));
	tfBroadcaster.sendTransform(tf_body);*/



   	ros::Rate rate(100);
   	bool status = ros::ok();;
   	while(status)
   	{
		ros::spinOnce();

		if (newLaserCloudFront & newOdomtry &
		    fabs(timeOdometry - timeLaserCloudFront) < 0.02)
		{
			if (firstFlag)
			{
				all_pointcloud->clear();
				firstFlag = false;
				continue; 
			}
			pcl::PointCloud<pcl::PointXYZ>::Ptr temp(new pcl::PointCloud<pcl::PointXYZ>());


			timeCloud = timeLaserCloudFront;
			Eigen::Affine3d tran = Eigen::Affine3d::Identity();
  			tran.translation() <<pose_Translation[0], pose_Translation[1], pose_Translation[2];
  			Eigen::Quaterniond quat(pose_Translation[6], pose_Translation[3], pose_Translation[4],pose_Translation[5]);
  			tran.rotate(quat);
  			pcl::transformPointCloud(*front_pointcloud, *temp, tran);
  			*all_pointcloud += *temp;
  			//publish message
  			sensor_msgs::PointCloud2 LaserCloudFull;
         			pcl::toROSMsg(*all_pointcloud, LaserCloudFull);
         			LaserCloudFull.header.stamp = ros::Time().fromSec(timeLaserCloudFront);
         			LaserCloudFull.header.frame_id = "/map";
         			pubLaserCloudFull.publish(LaserCloudFull);

         			sensor_msgs::PointCloud2 LaserCloud;
         			pcl::toROSMsg(*front_pointcloud, LaserCloud);
         			LaserCloud.header.stamp = ros::Time().fromSec(timeLaserCloudFront);
         			LaserCloud.header.frame_id = "/body";
         			pubLaserCloud.publish(LaserCloud);


         			tf::Vector3 translation;
			tf::Quaternion rotate;
			//tf body
			translation = tf::Vector3(pose_Translation[0], -pose_Translation[1], -pose_Translation[2]);
			rotate = tf::Quaternion( pose_Translation[3], -pose_Translation[4], -pose_Translation[5], pose_Translation[6]);
			/*tf_body.stamp_ = ros::Time().fromSec(timeLaserCloudFront);
   			tf_body.setOrigin(translation);
			tf_body.setRotation(rotate);
			tfBroadcaster.sendTransform(tf_body);*/
			transforms.setOrigin(translation);
			transforms.setRotation(rotate);
			tfBroadcaster.sendTransform(tf::StampedTransform(   transforms,
										ros::Time().fromSec(timeLaserCloudFront),
										"/map",
										"/body"));

			//tf  front view
			translation = tf::Vector3(0,0,0.15);
			rotate.setRPY(0,0,0);
			transforms.setOrigin(translation);
			transforms.setRotation(rotate);
			tfBroadcaster.sendTransform(tf::StampedTransform(   transforms,
										ros::Time().fromSec(timeLaserCloudFront),
										"/body",
										"/base_link"));


			
		}
		status = ros::ok();
		rate.sleep();
   	}
   	front_pointcloud->clear();
   	all_pointcloud->clear();
}