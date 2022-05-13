#include "follow_path_base.hpp"
#include "as2_msgs/msg/pose_stamped_with_id.hpp"
#include "as2_msgs/srv/send_trajectory_waypoints.hpp"
#include "as2_msgs/srv/set_speed.hpp"
#include "as2_core/names/services.hpp"

#include "as2_core/synchronous_service_client.hpp"

namespace follow_path_plugins
{
    class FollowPathTraj : public follow_path_base::FollowPathBase
    {
        using YAW_MODE = as2_msgs::msg::TrajectoryWaypointsWithID;
        using SyncSetSpeed = as2::SynchronousServiceClient<as2_msgs::srv::SetSpeed>;
        using SyncSendTrajWayp = as2::SynchronousServiceClient<as2_msgs::srv::SendTrajectoryWaypoints>;
    public:
        rclcpp_action::GoalResponse onAccepted(const std::shared_ptr<const as2_msgs::action::FollowPath::Goal> goal) override
        {
            auto req_speed = as2_msgs::srv::SetSpeed::Request();
            auto resp_speed = as2_msgs::srv::SetSpeed::Response();

            req_speed.speed.speed = goal->trajectory_waypoints.max_speed;

            auto set_traj_speed_cli = SyncSetSpeed(as2_names::services::motion_reference::set_traj_speed);
            if (!set_traj_speed_cli.sendRequest(req_speed, resp_speed, 1))
            {
                return rclcpp_action::GoalResponse::REJECT;
            }

            auto req_traj = as2_msgs::srv::SendTrajectoryWaypoints::Request();
            auto resp_traj = as2_msgs::srv::SendTrajectoryWaypoints::Response();
            as2_msgs::msg::TrajectoryWaypoints trajectory_waypoints = goal->trajectory_waypoints;

            // Populate Waypoints queue
            for (auto &pose : trajectory_waypoints.poses)
            {
                waypoints_.emplace_back(Eigen::Vector3d(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z));
                as2_msgs::msg::PoseStampedWithID pose_with_id;
                pose_with_id.pose = pose.pose;
                req_traj.waypoints.poses.emplace_back(pose_with_id);
            }
            req_traj.waypoints.yaw_mode = goal->trajectory_waypoints.yaw_mode;

            auto send_traj_wayp_cli = SyncSendTrajWayp(as2_names::services::motion_reference::send_traj_wayp);
            if (!send_traj_wayp_cli.sendRequest(req_traj, resp_traj, 1))
            {

                return rclcpp_action::GoalResponse::REJECT;
            }

            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }

        rclcpp_action::CancelResponse onCancel(const std::shared_ptr<GoalHandleFollowPath> goal_handle) override
        {
            // TODO: since follow path is done by traj_gen + controllor, cancel has to be also handle by them
            waypoints_.clear();
            return rclcpp_action::CancelResponse::ACCEPT;
        }

        bool onExecute(const std::shared_ptr<GoalHandleFollowPath> goal_handle) override
        {
            rclcpp::Rate loop_rate(10);
            const auto goal = goal_handle->get_goal();
            auto feedback = std::make_shared<as2_msgs::action::FollowPath::Feedback>();
            auto result = std::make_shared<as2_msgs::action::FollowPath::Result>();

            // FIXME get next (first) from queue
            Eigen::Vector3d next_wayp = waypoints_.front();
            waypoints_.pop_front();

            Eigen::Vector3d position(this->current_pose_x_,
                                    this->current_pose_y_,
                                    this->current_pose_z_);
            float distance_to_next_waypoint = (position - next_wayp).norm();

            // Check if goal is done
            while (!checkGoalCondition())
            {
                if (goal_handle->is_canceling())
                {
                    result->follow_path_success = false;
                    goal_handle->canceled(result);
                    RCLCPP_INFO(node_ptr_->get_logger(), "Goal cannot be cancelled");
                    // return;
                }

                position = Eigen::Vector3d(this->current_pose_x_, this->current_pose_y_, this->current_pose_z_);
                distance_to_next_waypoint = (position - next_wayp).norm();

                if ( (distance_to_next_waypoint < goal_threshold_) && !waypoints_.empty() )
                {
                    next_wayp = waypoints_.front();
                    waypoints_.pop_front();
                }

                // RCLCPP_INFO(this->get_logger(), "Publish feedback");
                feedback->next_waypoint.x = next_wayp[0];
                feedback->next_waypoint.y = next_wayp[1];
                feedback->next_waypoint.z = next_wayp[2];
                feedback->remaining_waypoints = waypoints_.size();
                feedback->actual_distance_to_next_waypoint = distance_to_next_waypoint;
                feedback->actual_speed = actual_speed_;

                goal_handle->publish_feedback(feedback);

                loop_rate.sleep();
            }

            result->follow_path_success = true;
            waypoints_.clear();

            goal_handle->succeed(result);
            return true;
        }

    private:
        bool checkGoalCondition()
        {
            if (waypoints_.empty() && actual_speed_ < 0.1)
            {
                return true;
            }
            return false;
        }
    }; // FollowPathTraj class
} // follow_path_plugins namespace

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(follow_path_plugins::FollowPathTraj, follow_path_base::FollowPathBase)
