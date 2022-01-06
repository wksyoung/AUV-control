#include <controller_interface/controller.h>
#include <geometry_msgs/TwistStamped.h>
#include <hector_quadrotor_interface/quadrotor_interface.h>
#include <limits>
//#include <ros/subscriber.h>
#include <hector_quadrotor_interface/limiters.h>
#include <boost/thread/mutex.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <fstream>
#include <visualization_msgs/Marker.h>
#include <cstdlib>
#include <cmath>
#include <pluginlib/class_list_macros.h>
#include <hector_quadrotor_interface/helpers.h>
#include <Eigen/Dense>

namespace motion_controllers
{
    using namespace hector_quadrotor_interface;
    class Trajectory_Controller : public controller_interface::Controller<QuadrotorInterface>
    {
    public:
        Trajectory_Controller() : pose_command_valid_(false), twist_command_valid_(false), K(3,6){
            Eigen::Vector3d pk(-3.58, -3.58, -200);
            Eigen::Vector3d vk(-70, -70, -40);
            K <<  Eigen::MatrixXd(pk.asDiagonal()), Eigen::MatrixXd(vk.asDiagonal());
        }
        ~Trajectory_Controller(){
            //state_logger.close();
        }
        virtual bool init(QuadrotorInterface *interface,
                          ros::NodeHandle &root_nh,
                          ros::NodeHandle &controller_nh)
        {
            // get interface handles
            pose_ = interface->getPose();
            twist_ = interface->getTwist();
            motor_status_ = interface->getMotorStatus();
            // Initialize PID controllers
            //yaw_pid.init(ros::NodeHandle(controller_nh, "yaw"));

            position_limiter_.init(controller_nh);
            twist_limiter_.init(controller_nh);

            // Setup pose visualization marker output
            initMarker(root_nh.getNamespace());
            marker_publisher_ = root_nh.advertise<visualization_msgs::Marker>("command/pose_marker", 1);

            // Initialize command inputs
            pose_input_ = interface->addInput<PoseCommandHandle>("pose");
            twist_input_  = interface->addInput<TwistCommandHandle>("twist");
            accel_input_ = interface->addInput<AccelCommandHandle>("accel");
            // Add control outputs
            attitude_output_ = interface->addOutput<AttitudeCommandHandle>("attitude");
            heading_output_ = interface->addOutput<HeadingCommandHandle>("heading");
            thrust_output_ = interface->addOutput<ThrustCommandHandle>("thrust");

            //std::string path;
            //controller_nh.param<std::string>("io_logpath", path, "~/adplog.txt");
            root_nh.param("/robot/mass", mass_, 1.2);
            div = 0;
            //state_logger = std::ofstream(path);

            return true;
        }

        void reset()
        {
            // Set commanded pose to robot's current pose
            //updatePoseCommand(pose_->pose());

            pose_command_valid_ = false;
            twist_command_valid_= false;
        }

        virtual void starting(const ros::Time &time)
        {
            reset();
        }

        virtual void stopping(const ros::Time &time)
        {
            attitude_output_->stop();
            thrust_output_->stop();
            heading_output_->stop();
            pose_command_valid_ = false;
            twist_command_valid_ = false;
        }

        virtual void update(const ros::Time &time, const ros::Duration &period)
        {
            boost::mutex::scoped_lock lock(command_mutex_);
            /*
            if(++div < 6)
                return;
            else
                div = 0;*/

            // Get pose command input
            if (pose_input_->connected() && pose_input_->enabled()){
                updatePoseCommand(pose_input_->getCommand());
            }
            // Get twist command input
            if (twist_input_->connected() && twist_input_->enabled()){
                updateTwistCommand(twist_input_->getCommand());
            }
            // Get accelerate command input
            if (accel_input_->connected() && accel_input_->enabled()){
                updateAccelCommand(accel_input_->getCommand());
            }

            // check command timeout
            // TODO

			// Check if motors are running
            if (motor_status_->motorStatus().running == false) {
                if (pose_command_valid_  && twist_command_valid_) {
                    //ROS_INFO_NAMED("trajectory_controller", "Disabled position control while motors are not running.");
                }
                pose_command_valid_ = false;
				twist_command_valid_ = false;
            }
			if (pose_command_valid_ && twist_command_valid_) {
				attitude_output_->start();
            	thrust_output_->start();
                heading_output_->start();
    		} else {
				//reset();
      			attitude_output_->stop();
            	thrust_output_->stop();
                heading_output_->stop();
      			return;
    		}

            Pose pose = pose_->pose();
            Twist twist = twist_->twist();

            double yaw_command;
            {
                tf2::Quaternion q;
                double temp;
                tf2::fromMsg(pose_command_.orientation, q);
                tf2::Matrix3x3(q).getRPY(temp, temp, yaw_command);
                heading_output_->setCommand(yaw_command);
            }

            pose_command_.position = position_limiter_(pose_command_.position);
            twist_command_ = twist_limiter_(twist_command_);

            Eigen::VectorXd x(6);
            x << pose_->pose().position.x - pose_command_.position.x,
                    pose_->pose().position.y - pose_command_.position.y,
                    pose_->pose().position.z - pose_command_.position.z,
                    twist_->twist().linear.x - twist_command_.linear.x,
                    twist_->twist().linear.y - twist_command_.linear.y,
                    twist_->twist().linear.z - twist_command_.linear.z;
            const double gravity = 9.8065;
            Eigen::Vector3d uc = K * x;

            //state_logger << "x: " <<x.transpose() << ", u: " << uc.transpose() << std::endl;
            double mg = mass_*gravity;
            Eigen::Vector3d mge3(0, 0, mg);
            Eigen::Vector3d ddpd(accel_command_.linear.x, 
                                    accel_command_.linear.y, 
                                    accel_command_.linear.z);
            /*
             double uc_norm = uc.norm();
             if(uc_norm > mg)    {
                 uc = mg*uc / uc_norm;
                 uc_norm = mg;
             }
             Eigen::Vector3d vf = mge3;// + 9*ddpd;
             if(uc_norm > vf.norm()){
                ROS_WARN("singularity detected because uc.norm() > vf.norm()");
                return;
             }
            Eigen::Vector3d uf = vf + uc;*/
            if (uc(2) < 0.01 - mg)
                uc(2)  = 0.01 - mg;
            Eigen::Vector3d uf = mge3 + uc;

            hector_uav_msgs::ThrustCommand thrust_control;
            hector_uav_msgs::AttitudeCommand attitude_control;

            thrust_control.thrust = uf.norm();

            Eigen::Vector3d w = uf / uf.norm();
            attitude_control.roll = atan2(-w(1), w(2));
            attitude_control.pitch = asin(w(0));

            // pass down time stamp from twist command
    		attitude_control.header.stamp = time;
    		thrust_control.header.stamp = time;

            // set twist output
            attitude_output_->setCommand(attitude_control);
            thrust_output_->setCommand(thrust_control);
        }
    private:
        void updatePoseCommand(const geometry_msgs::Pose &new_pose)
        {
            {
                pose_command_.position = new_pose.position;
                // Strip non-yaw components from orientation
                tf2::Quaternion q;
                double roll, pitch, yaw;
                tf2::fromMsg(new_pose.orientation, q);
                tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
                q.setRPY(0, 0, yaw);
                pose_command_.orientation = tf2::toMsg(q);
                pose_command_valid_ = true;
            }
            pose_marker_.pose = pose_command_;
            marker_publisher_.publish(pose_marker_);
        }

        void updateTwistCommand(const geometry_msgs::Twist &command){
            twist_command_ = command;
            twist_command_valid_ = true;
        }

        void updateAccelCommand(const geometry_msgs::Accel &command){
            accel_command_ = command;
            accel_command_valid_ = true;
        }

        void initMarker(std::string name)
        {
            pose_marker_.header.frame_id = "world";
            pose_marker_.ns = name;
            pose_marker_.id = 0;
            pose_marker_.type = visualization_msgs::Marker::ARROW;
            pose_marker_.scale.x = 0.15;
            pose_marker_.scale.y = pose_marker_.scale.z = 0.03;
            pose_marker_.color.r = 0.5;
            pose_marker_.color.g = 0.5;
            pose_marker_.color.r = 0.5;
            pose_marker_.color.a = 1.0;
        }

        PoseHandlePtr pose_;
        TwistHandlePtr twist_;
        MotorStatusHandlePtr motor_status_;

        PoseCommandHandlePtr pose_input_;
        TwistCommandHandlePtr twist_input_;
        AccelCommandHandlePtr accel_input_;
        HeadingCommandHandlePtr heading_output_;
        AttitudeCommandHandlePtr attitude_output_;
        ThrustCommandHandlePtr thrust_output_;

        //TwistCommandHandlePtr twist_limit_input_;
        hector_quadrotor_interface::PointLimiter position_limiter_;
        hector_quadrotor_interface::TwistLimiter twist_limiter_;

        //ros::Subscriber pose_subscriber_, twist_subscriber_;
        ros::Publisher marker_publisher_;

        visualization_msgs::Marker pose_marker_;

        geometry_msgs::Pose pose_command_;
        geometry_msgs::Twist twist_command_;
        geometry_msgs::Accel accel_command_;
        bool pose_command_valid_, twist_command_valid_, accel_command_valid_;

        double mass_;
        double inertia_[3];
        uint16_t div;
        Eigen::MatrixXd K;

        boost::mutex command_mutex_;
    };
}//namespace auv_controllers

PLUGINLIB_EXPORT_CLASS(motion_controllers::Trajectory_Controller, controller_interface::ControllerBase)
