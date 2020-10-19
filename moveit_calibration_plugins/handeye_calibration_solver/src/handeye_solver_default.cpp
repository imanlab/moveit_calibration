/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019,  Intel Corporation.
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
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
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

/* Author: Yu Yan */

#include <moveit/handeye_calibration_solver/handeye_solver_default.h>

namespace moveit_handeye_calibration
{
const std::string LOGNAME = "handeye_solver_default";

void HandEyeSolverDefault::initialize()
{
  solver_map_.insert("Daniilidis1999", cv::HandEyeCalibrationMethod::CALIB_HAND_EYE_DANIILIDIS);
  solver_map_.insert("ParkMartin1994", cv::HandEyeCalibrationMethod::CALIB_HAND_EYE_PARK);
  solver_map_.insert("TsaiLenz1989", cv::HandEyeCalibrationMethod::CALIB_HAND_EYE_DANIILIDIS);
  solver_map_.insert("HoraudDornaika1995", cv::HandEyeCalibrationMethod::CALIB_HAND_EYE_HORAUD);
  solver_map_.insert("AndreffEtAl1999", cv::HandEyeCalibrationMethod::CALIB_HAND_EYE_ANDREFF);

  for (const auto& solver_pair : solver_map_)
  {
    solver_names_.push_back(solver_pair.first);
  }

  camera_robot_pose_ = Eigen::Isometry3d::Identity();
}

const std::vector<std::string>& HandEyeSolverDefault::getSolverNames() const
{
  return solver_names_;
}

const Eigen::Isometry3d& HandEyeSolverDefault::getCameraRobotPose() const
{
  return camera_robot_pose_;
}

bool HandEyeSolverDefault::solve(const std::vector<Eigen::Isometry3d>& effector_wrt_world,
                                 const std::vector<Eigen::Isometry3d>& object_wrt_sensor, SensorMountType setup,
                                 const std::string& solver_name)
{
  // Check the size of the two sets of pose sample equal
  if (effector_wrt_world.size() != object_wrt_sensor.size())
  {
    ROS_ERROR_STREAM_NAMED(LOGNAME, "The sizes of the two input pose sample "
                                    "vectors are not equal: "
                                    "effector_wrt_world.size() = "
                                        << effector_wrt_world.size()
                                        << " and object_wrt_sensor.size() = " << object_wrt_sensor.size());
    return false;
  }

  auto it = std::find(solver_names_.begin(), solver_names_.end(), solver_name);
  if (it == solver_names_.end())
  {
    ROS_ERROR_STREAM_NAMED(LOGNAME, "Unknown handeye solver name: " << solver_name);
    return false;
  }

  size_t num_poses = effector_wrt_world.size();

  // OpenCV inputs
  std::vector<cv::Mat> R_gripper_to_base(num_poses);
  std::vector<cv::Mat> t_gripper_to_base(num_poses);
  std::vector<cv::Mat> R_target_to_cam(num_poses);
  std::vector<cv::Mat> t_target_to_cam(num_poses);
  cv::Mat R_cam_to_gripper;
  cv::Mat t_cam_to_gripper;

  for (size_t i = 0; i < num_poses; ++i)
  {
    cv::eigen2cv(effector_wrt_world[i].linear(), R_gripper_to_base[i]);
    cv::eigen2cv(effector_wrt_world[i].translation(), t_gripper_to_base[i]);
    if (setup == EYE_IN_HAND)
    {
      cv::eigen2cv(object_wrt_sensor[i].linear(), R_target_to_cam[i]);
      cv::eigen2cv(object_wrt_sensor[i].translation(), t_target_to_cam[i]);
    }
    else if (setup == EYE_TO_HAND)
    {
      cv::eigen2cv(object_wrt_sensor[i].inverse().linear(), R_target_to_cam[i]);
      cv::eigen2cv(object_wrt_sensor[i].inverse().translation(), t_target_to_cam[i]);
    }
  }
  cv::calibrateHandEye(R_gripper_to_base, t_gripper_to_base, R_target_to_cam, t_target_to_cam, R_cam_to_gripper,
                       t_cam_to_gripper, solver_map_(solver_name));
  if (setup == EYE_TO_HAND)
  {
    // TODO: I think we just got the gripper-to-target transform
  }

  Eigen::Matrix4d T_cam_to_gripper_eig = Eigen::Matrix4d::Identity();
  cv::cv2eigen(R_cam_to_gripper, T_cam_to_gripper_eig.block<3, 3>(0, 0));
  cv::cv2eigen(t_cam_to_gripper, T_cam_to_gripper_eig.block<3, 1>(0, 3));

  camera_robot_pose_ = Eigen::Isometry3d(T_cam_to_gripper_eig);

  return true;
}
}  // namespace moveit_handeye_calibration
