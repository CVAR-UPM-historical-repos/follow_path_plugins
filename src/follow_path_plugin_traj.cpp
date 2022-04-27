#include "follow_path_base.hpp"
#include "as2_msgs/msg/trajectory_waypoints.hpp"

namespace follow_path_plugins
{
    class FollowPathTraj : public follow_path_base::FollowPathBase
    {
    public:
        void ownInit(as2::Node *node_ptr) override
        {
            traj_waypoints_pub_ = node_ptr_->create_publisher<as2_msgs::msg::TrajectoryWaypoints>(
                node_ptr_->generate_global_name(as2_names::topics::motion_reference::wayp), 
                as2_names::topics::motion_reference::qos_wp);
        }


        rclcpp_action::GoalResponse onAccepted(const std::shared_ptr<const as2_msgs::action::FollowPath::Goal> goal) override
        {
            as2_msgs::msg::TrajectoryWaypoints trajectory_waypoints = goal->trajectory_waypoints;
            // Populate Waypoints queue
            for(auto it = trajectory_waypoints.poses.begin(); it != trajectory_waypoints.poses.end(); ++it ) {
                waypoints_.push_back(Eigen::Vector3d(it->pose.position.x, it->pose.position.y, it->pose.position.z));
            }

            // Assign the goal to the Eigen Vector
            traj_waypoints_pub_->publish(trajectory_waypoints);
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
    private:
        rclcpp::Publisher<as2_msgs::msg::TrajectoryWaypoints>::SharedPtr traj_waypoints_pub_;  

    }; // FollowPathTraj class
} // follow_path_plugins namespace

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(follow_path_plugins::FollowPathTraj, follow_path_base::FollowPathBase)