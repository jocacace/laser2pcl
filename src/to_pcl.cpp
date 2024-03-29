/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2024, Jonathan Cacace <jonathan.cacace@gmail.com>
*	 EURECAT
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>


using namespace Eigen;

class L2PCL {
     public:
        L2PCL();
        //void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
        void scanCallback( sensor_msgs::LaserScan scan);
     private:
        ros::NodeHandle node_;
        laser_geometry::LaserProjection projector_;
        tf::TransformListener tfListener_;

        ros::Publisher point_cloud_publisher_;
        ros::Subscriber scan_sub_;
};

L2PCL::L2PCL(){
        scan_sub_ = node_.subscribe<sensor_msgs::LaserScan> ("/scan", 100, &L2PCL::scanCallback, this);
        point_cloud_publisher_ = node_.advertise<sensor_msgs::PointCloud2> ("/cloud", 1, false);
        //tfListener_.setExtrapolationLimit(ros::Duration(1000.5));

}


Matrix3d XYZ2R(Vector3d angles) {
    
    Matrix3d R = Matrix3d::Zero(); 
    Matrix3d R1 = Matrix3d::Zero(); 
    Matrix3d R2 = Matrix3d::Zero(); 
    Matrix3d R3 = Matrix3d::Zero();

    float cos_phi = cos(angles[0]);
    float sin_phi = sin(angles[0]);
    float cos_theta = cos(angles[1]);
    float sin_theta = sin(angles[1]);
    float cos_psi = cos(angles[2]);
    float sin_psi = sin(angles[2]);

    R1  << 1, 0      , 0, 
                0, cos_phi, -sin_phi, 
                0, sin_phi, cos_phi;

    R2  << cos_theta , 0, sin_theta, 
                0        , 1, 0       , 
                -sin_theta, 0, cos_theta;

    R3  << cos_psi, -sin_psi, 0, 
                sin_psi, cos_psi , 0,
                0      , 0       , 1;

    R = R1*R2*R3;

    return R;
}

void L2PCL::scanCallback(sensor_msgs::LaserScan  scan){

		// ROS PointCLoud2
    sensor_msgs::PointCloud2 cloud;
    
    try {
    
    	// From lidar to ROS PointCloud2
		  projector_.transformLaserScanToPointCloud("lidar_link", scan, cloud, tfListener_);

			// From ROS PointCloud2 to pcl PointCloud2		  
		  pcl::PCLPointCloud2 pcl_pc2;
			pcl_conversions::toPCL(cloud, pcl_pc2);
			 
			// From pcl PointCloud2 to pcl PointCloud
			pcl::PointCloud<pcl::PointXYZ> pcl1_cloud;
  		pcl::fromPCLPointCloud2(pcl_pc2, pcl1_cloud);
			
			double roll = 0.0;
			double pitch = 0.0;
			
			Eigen::Matrix3d R = XYZ2R( Eigen::Vector3d( 0.5, 0.0, 1.5 ) );
			
			// Generate rotation matrix for the pointcloud 
			Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
			
			
			transform_1 (0,0) = R(0,0);
			transform_1 (0,1) = R(0,1);
			transform_1 (0,2) = R(0,2);
			transform_1 (1,0) = R(1,0);
			transform_1 (1,1) = R(1,1);
			transform_1 (1,2) = R(1,2);
			transform_1 (2,0) = R(2,0);
			transform_1 (2,1) = R(2,1);
			transform_1 (2,2) = R(2,2);
		
			
			
			printf ("Rotate by using a Matrix4f\n");
			std::cout << transform_1 << std::endl;
			
			// Rotate pcl::PointCLoud
			pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
			pcl::transformPointCloud (pcl1_cloud, *transformed_cloud, transform_1);

			sensor_msgs::PointCloud2 object_msg;
			pcl::toROSMsg(*transformed_cloud.get(),object_msg );
			
			point_cloud_publisher_.publish(object_msg);
		  
		}
		catch (tf::TransformException &ex) {
		  ROS_ERROR("%s", ex.what());
		  //ros::Duration(0.05).sleep();
		}
		
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "L2PCL");

    L2PCL filter;

    ros::spin();

    return 0;
}
