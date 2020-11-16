#include "rars_motion_execution/do_motion_exe_server.h"
#include <rars_msgs/DoMotionExecutionAction.h>
#include <rars_msgs/GenerateRobotProgram.h>
#include <std_srvs/Trigger.h>
#include <std_srvs/Empty.h>
#include <ros/console.h>
#include <string>
#include <trajectory_msgs/JointTrajectory.h>
#include <geometry_msgs/TransformStamped.h>
#include <mutable_transform_publisher/SetTransform.h>
#include <yak_ros_msgs/GenerateMesh.h>

namespace rars_motion_execution
{
const static std::string GENERATE_ROBOT_PROGRAM_SERVICE = "generate_robot_program";

MotionExecutionServer::MotionExecutionServer(ros::NodeHandle& nh)
  : nh_(nh)
  , as_(nh, "do_motion_execution", false)
  , prg_plc_interface_(nh_)
  , tsdf_plc_interface_(nh_)
  , yak_plc_interface_(nh_)
  , pr_sub_def_(this)
  , tsdf_sub_def_(this)
  , yak_sub_def_(this)
{
  // register action
  as_.registerGoalCallback(boost::bind(&MotionExecutionServer::executeCB, this));

  // Connect to the plc and set up the data subscription
  prg_plc_interface_.init();
  tsdf_plc_interface_.init();
  yak_plc_interface_.init();

  pr_sub_ = prg_plc_interface_.client.CreateSubscription(1, pr_sub_def_);
  tsdf_sub_ = tsdf_plc_interface_.client.CreateSubscription(1, tsdf_sub_def_);
  yak_sub_ = yak_plc_interface_.client.CreateSubscription(1, yak_sub_def_);

  prg_running_subscription_handle_ =
      pr_sub_->SubscribeDataChange(prg_plc_interface_.client.GetNode(rars_plc_interface::node_ids::STATUS_PRG_RUNNING));
  tsdf_subscription_handle_ =
      tsdf_sub_->SubscribeDataChange(tsdf_plc_interface_.client.GetNode(rars_plc_interface::node_ids::REQ_TSDF));
  yak_subscription_handle_ =
      yak_sub_->SubscribeDataChange(yak_plc_interface_.client.GetNode(rars_plc_interface::node_ids::REQ_YAK));

  ros::Duration(0.5).sleep();

  //  LS conversion call
  ls_client_ = nh_.serviceClient<rars_msgs::GenerateRobotProgram>(GENERATE_ROBOT_PROGRAM_SERVICE);
  bool success = true;
  success &= ls_client_.waitForExistence(ros::Duration(60));
  if (!success)
  {
    ROS_ERROR("Unable to contact ros_to_ls server");
  }

  // Allow the server to respond to requests
  ROS_INFO("connected to LS server");
  as_.start();
  return;
}

MotionExecutionServer::~MotionExecutionServer()
{
  pr_sub_->UnSubscribe(prg_running_subscription_handle_);
  tsdf_sub_->UnSubscribe(tsdf_subscription_handle_);
  yak_sub_->UnSubscribe(yak_subscription_handle_);

  return;
}

void MotionExecutionServer::executeCB()
{
  const rars_msgs::DoMotionExecutionGoalConstPtr& goal = as_.acceptNewGoal();

  rars_msgs::DoMotionExecutionResult result;
  if (as_.isPreemptRequested())
  {
    // if another message arrives
    result.success = false;
    ROS_INFO("Preempt");
    result.error_msg = "Preempt requested";
    as_.setPreempted(result);
    return;
  }

  tsdf_sub_def_.tsdf_updates = goal->tsdf_params;
  tsdf_sub_def_.tsdf_frame_transform = goal->tsdf_frame_transform;
  yak_sub_def_.res_dir = goal->results_directory;

  // packing messages field by field:
  request.process_type.command = 0;  // TODO: handle other process_types
  request.instructions = goal->instructions;

  bool success = ls_client_.call(request, response);
  if (!success || !response.success)
  {
    result.success = false;
    result.error_msg = "LS Conversion failed";
    as_.setAborted(result);
    return;
  }

  // Use the PLC to start the program
  if (prg_plc_interface_.writeTag(rars_plc_interface::node_ids::CTRL_JOB_NUM, static_cast<std::uint16_t>(10)))
  {
    if (prg_plc_interface_.writeTag(rars_plc_interface::node_ids::CTRL_JOB_LOAD, true))
    {
      bool status_job_loaded = false;
      bool status_prod_mode = false;
      bool read_correctly = false;
      for (int i = 0; i < 4; ++i)
      {
        read_correctly = prg_plc_interface_.readTag(rars_plc_interface::node_ids::STATUS_JOB_LOADED, status_job_loaded);
        read_correctly &= prg_plc_interface_.readTag(rars_plc_interface::node_ids::STATUS_PROD_MODE, status_prod_mode);
        if (read_correctly && status_job_loaded && status_prod_mode)
          break;
        else
          ros::Duration(0.5).sleep();
      }
      if (read_correctly && status_job_loaded && status_prod_mode)
      {
        if (!prg_plc_interface_.writeTag(rars_plc_interface::node_ids::CTRL_START, true))
        {
          ROS_ERROR("Could not set Ctrl.Start");
          rars_msgs::DoMotionExecutionResult result;
          result.success = false;
          result.error_msg = "Could not set Ctrl.Start";
          as_.setAborted(result);
        }
      }
      else
      {
        ROS_ERROR("Status.JobLoaded and Status.ProdMode were not set or read correctly to allow execution");
        rars_msgs::DoMotionExecutionResult result;
        result.success = false;
        result.error_msg = "Status.JobLoaded and Status.ProdMode were not set or read correctly to allow execution";
        as_.setAborted(result);
      }
    }
    else
    {
      ROS_ERROR("Ctrl.JobLoad was not set correctly");
      rars_msgs::DoMotionExecutionResult result;
      result.success = false;
      result.error_msg = "Ctrl.JobLoad was not set correctly";
      as_.setAborted(result);
    }
  }
  else
  {
    ROS_ERROR("Ctrl.JobNum was not set correctly");
    rars_msgs::DoMotionExecutionResult result;
    result.success = false;
    result.error_msg = "Ctrl.JobNum was not set correctly";
    as_.setAborted(result);
  }

  /* @brief
   * At this point, we exit this function. The action call will remain 'running' even after the
   * termination of this function, until something calls either setAborted, setPreempted, or
   * setSucceeded.  When the robot is done executing, it will change the PrgRunning tag.  This will
   * trigger the below subscription callback.  That callback will handle the correct returning of
   * an action result.
   */
  ROS_INFO("End of Callback");
  return;
}

MotionExecutionServer::TSDFSubscriptionDefinition::TSDFSubscriptionDefinition(MotionExecutionServer* srvr) : srvr_(srvr)
{
  if (srvr_ == nullptr)
  {
    ROS_ERROR("Received null pointer while instantiating TSDFSubscriptionDefinition");
  }

  reset_params_client_ = srvr_->nh_.serviceClient<yak_ros_msgs::UpdateKinFuParams>("yak_node/update_params");

  set_tsdf_frame_client_ = srvr_->nh_.serviceClient<mutable_transform_publisher::SetTransform>("set_transform");

  clear_volume_client_ = srvr_->nh_.serviceClient<std_srvs::Trigger>("yak_node/reset_tsdf");

  reset_octomap_client_ = srvr_->nh_.serviceClient<std_srvs::Empty>("/octomap_server/reset");

  return;
}

void MotionExecutionServer::TSDFSubscriptionDefinition::DataChange(uint32_t handle,
                                                                   const OpcUa::Node& node,
                                                                   const OpcUa::Variant& value,
                                                                   OpcUa::AttributeId attr)
{
  // Ensure the class was properly instantiated
  if (srvr_ == nullptr)
  {
    ROS_ERROR("srvr_ is nullptr");
    return;
  }

  if (srvr_->as_.isActive())
  {
    bool tsdf_requested = false;
    if (!srvr_->tsdf_plc_interface_.readTag(rars_plc_interface::node_ids::REQ_TSDF, tsdf_requested))
    {
      ROS_ERROR("Could not read tag value");

      // Set Failed, the robot is in an unknown state
      rars_msgs::DoMotionExecutionResult result;
      result.success = false;
      result.error_msg = "Could not read current state of prg_running tag.  This may be very bad.";
      srvr_->as_.setAborted(result);
    }
    // If tag is changed to True
    if (tsdf_requested)
    {
      ROS_ERROR("TSDF requested");
      // Create the service call
      yak_ros_msgs::UpdateKinFuParams reset_params_srv;
      reset_params_srv.request.kinfu_params = tsdf_updates;
      std::vector<int> reset_params{ 4, 5 };
      reset_params_srv.request.params_to_update = reset_params;

      if (!reset_params_client_.call(reset_params_srv) || !reset_params_srv.response.success)
      {
        ROS_ERROR("Failed to set tsdf volume params");
        return;
      }

      // set transform for the tsdf frame
      mutable_transform_publisher::SetTransform set_transform_srv;
      set_transform_srv.request.transform = tsdf_frame_transform;

      if (!set_tsdf_frame_client_.call(set_transform_srv) || !set_transform_srv.response.was_replaced)
      {
        ROS_ERROR("Failed to set transform for tsdf_frame");
        return;
      }

      // clear the tsdf volume for the next scan
      std_srvs::Trigger clear_volume_srv;

      if (!clear_volume_client_.call(clear_volume_srv) || !clear_volume_srv.response.success)
      {
        ROS_ERROR("Failed to clear tsdf volume");
      }

      ROS_INFO("TSDF Config Complete");

      // ak flag
      if (!srvr_->tsdf_plc_interface_.writeTag(rars_plc_interface::node_ids::REQ_TSDF_AK, true))
      {
        ROS_ERROR("Could not correctly reset Ctrl.TSDFReqAk");
      }
    }
    else
    {
      ROS_INFO("TSDF Req was false");
    }
    ROS_INFO("Exiting TSDF datachange Callback");
  }
  return;
}

MotionExecutionServer::YakSubscriptionDefinition::YakSubscriptionDefinition(MotionExecutionServer* srvr) : srvr_(srvr)
{
  if (srvr_ == nullptr)
  {
    ROS_ERROR("Received null pointer while instantiating YakSubscriptionDefinition");
  }

  // generate the mesh from the scan
  generate_mesh_client_ = srvr->nh_.serviceClient<yak_ros_msgs::GenerateMesh>("yak_node/generate_mesh");

  return;
}

void MotionExecutionServer::YakSubscriptionDefinition::DataChange(uint32_t handle,
                                                                  const OpcUa::Node& node,
                                                                  const OpcUa::Variant& value,
                                                                  OpcUa::AttributeId attr)
{
  // Ensure the class was properly instantiated
  if (srvr_ == nullptr)
  {
    ROS_ERROR("srvr_ is nullptr");
    return;
  }

  if (srvr_->as_.isActive())
  {
    bool yak_requested = false;
    if (!srvr_->yak_plc_interface_.readTag(rars_plc_interface::node_ids::REQ_YAK, yak_requested))
    {
      ROS_ERROR("Could not read yak tag value");

      // Set Failed, the robot is in an unknown state
      rars_msgs::DoMotionExecutionResult result;
      result.success = false;
      result.error_msg = "Could not read current state of prg_running tag.  This may be very bad.";
      srvr_->as_.setAborted(result);
    }

    if (yak_requested)
    {
      ROS_ERROR("Yak requested");
      // generate the mesh from the scan
      yak_ros_msgs::GenerateMesh generate_mesh_srv;
      generate_mesh_srv.request.results_dir = res_dir;
      if (!generate_mesh_client_.call(generate_mesh_srv) || !generate_mesh_srv.response.success)
      {
        ROS_ERROR_STREAM("Failed to generate mesh");
        return;
      }
      srvr_->results_out = generate_mesh_srv.response.results_path;

      ROS_ERROR("Marching cubes complete");

      // ak flag
      if (!srvr_->yak_plc_interface_.writeTag(rars_plc_interface::node_ids::REQ_YAK_AK, true))
      {
        ROS_ERROR("Could not correctly reset Ctrl.YakReqAk");
      }
    }
    else
    {
      ROS_INFO("Yak req is false");
    }
    ROS_INFO("Exiting YAK datachange Callback");
  }
  return;
}

MotionExecutionServer::PrgRunningSubscriptionDefinition::PrgRunningSubscriptionDefinition(MotionExecutionServer* srvr)
  : srvr_(srvr)
{
  if (srvr_ == nullptr)
  {
    ROS_ERROR("Received null pointer while instantiating PrgRunningSubscriptionDefinition");
  }
}

void MotionExecutionServer::PrgRunningSubscriptionDefinition::DataChange(uint32_t handle,
                                                                         const OpcUa::Node& node,
                                                                         const OpcUa::Variant& value,
                                                                         OpcUa::AttributeId attr)
{
  // Ensure the class was properly instantiated
  if (srvr_ == nullptr)
  {
    ROS_ERROR("srvr_ is nullptr");
    return;
  }

  if (srvr_->as_.isActive())
  {
    bool prg_idle = false;

    if (!srvr_->prg_plc_interface_.readTag(rars_plc_interface::node_ids::STATUS_PROG_IDLE, prg_idle))
    {
      ROS_ERROR("Could not read tag value");

      // Set Failed, the robot is in an unknown state
      rars_msgs::DoMotionExecutionResult result;
      result.success = false;
      result.error_msg = "Could not read current state of prg_running tag.  This may be very bad.";
      srvr_->as_.setAborted(result);
    }
    else if (prg_idle)
    {
      if (!srvr_->prg_plc_interface_.writeTag(rars_plc_interface::node_ids::CTRL_JOB_LOAD, false))
      {
        ROS_ERROR("Could not correctly reset Ctrl.JobLoad");
      }
      // Set succeeded
      rars_msgs::DoMotionExecutionResult result;
      result.results_path = srvr_->results_out;
      result.success = true;
      result.error_msg = "";
      srvr_->as_.setSucceeded(result);

      ROS_ERROR("Motion execution complete");
    }
  }
  return;
}
}  // namespace rars_motion_execution
