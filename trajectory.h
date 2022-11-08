#ifndef __TRAJECTORY_H
#define __TRAJECTORY_H

#include <Eigen/Dense>
#include <vector>
#include <iostream>


namespace pt
{

template<typename T>
class Trajectory
{
public:
	using DataType = T;
	using PoseType = typename Eigen::Matrix<DataType, 2, 1>;

	Trajectory()
	{

	}

	~Trajectory()
	{

	}

	void addPose( const PoseType& pose ) 
	{
		return poses.push_back( pose );
	}

	const PoseType& operator[]( const int i ) const
	{
		return poses[i];
	}

	const bool isEmpty() const
	{
		return poses.empty();
	}

	const int getSize() const
	{
		return poses.size();
	}
	
	void clearAll() 
	{
		return poses.clear();
	}
	
	void updatePoses( const std::vector<PoseType>& poses_ )
	{
		poses = poses_;
	}
	
	const std::vector<PoseType>& getPoses() const
	{
		return poses;
	}

private:
	std::vector<PoseType> poses;
	
};

}

#endif
