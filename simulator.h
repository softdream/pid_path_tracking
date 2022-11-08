#ifndef __SIMULATOR_H
#define __SIMULATOR_H

#include "path_tracking.h"
#include "trajectory.h"

#include <opencv2/opencv.hpp>

#define WORLD_WIDTH 800
#define WORLD_HEIGHT 800


namespace pt
{

class Simulator
{
public:
	using Vector3 = typename Eigen::Matrix<float, 3, 1>;

	Simulator()
	{

	}

	~Simulator()
	{

	}

	void initTrajectory()
	{
		for( float x = 0; x < 800; x ++ ){
			float y = ::sin( x * 0.01 ) * 100 + WORLD_HEIGHT / 2;
	
			//std::cout<<"( "<<x<<", "<<y<<" )"<<std::endl;

			trajectory.addPose( Trajectory<float>::PoseType( x, y ) );

		}
		
		drawTrajectory();
		cv::imshow( "map", map );
		cv::waitKey(0);
	}

	void process()
	{
		for( int i = 1; i < trajectory.getSize(); i ++ ){
			float target_yaw = tracking.cacuYaw( trajectory[i], trajectory[i + 1] );	
			std::cout<<"target yaw = "<<target_yaw<<std::endl;
			std::cout<<"current yaw = "<<robot_pose[2]<<std::endl;	
			std::cout<<"trajectory pose : "<<trajectory[i][0]<<", "<<trajectory[i][1]<<std::endl;
			std::cout<<"robot pose : "<<robot_pose[0]<<", "<<robot_pose[1]<<std::endl;	

			float u_yaw = tracking.yawPidProcess( robot_pose[2], target_yaw );
			std::cout<<"yaw pid out u_yaw = "<<u_yaw<<std::endl;
			
			Tracking<float>::Vector2 curr_pose( robot_pose[0], robot_pose[1] );
			float u_pose = tracking.yPosePidProcess( robot_pose[1], trajectory[i][1] );

			float u_x_pose = tracking.xPosePidProcess( robot_pose[0], trajectory[i][0] );
	
			float v = u_pose + u_x_pose;
			float w = 0 + u_yaw;

			motionModel( v, w );
			drawRobotPose();
			cv::imshow( "map", map );
			cv::waitKey(50);

			//cv::imwrite( std::to_string( i ) + ".png", map );
			std::cout<<std::endl;
		}
	}

	void motionModel( const float v, const float w )
	{
		robot_pose[2] += w * dt;
		robot_pose[0] += v * ::cos( robot_pose[2] ) * dt;
		robot_pose[1] += v * ::sin( robot_pose[2] ) * dt;
	}

private:
	void drawRobotPose()
	{
		cv::circle( map, cv::Point( robot_pose[0], robot_pose[1] ), 4, cv::Scalar( 0, 0, 255 ), -1 );

	}
	
	void drawTrajectory()
	{
		for( int i = 0; i < trajectory.getSize() - 1; i ++ ){
			cv::line( map, cv::Point( trajectory[i][0], trajectory[i][1] ), cv::Point( trajectory[i + 1][0], trajectory[i + 1][1] ), cv::Scalar( 255, 255, 0 ), 2 );
		
		}
	
	}

private:
	Tracking<float> tracking;

	Trajectory<float> trajectory;

	cv::Mat map = cv::Mat( WORLD_WIDTH, WORLD_HEIGHT, CV_8UC3, cv::Scalar(255, 255, 255 ) );

	const float dt = 0.5;
	Vector3 robot_pose = Vector3( 0, 400, 0.75 ); // initial pose of the robot
};

}

#endif
