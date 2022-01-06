#include <ros/ros.h>

#include <controller_interface/controller.h>
//#include <control_toolbox/pid.h>

#include <geometry_msgs/Wrench.h>
#include <pluginlib/class_list_macros.h>
#include <hector_quadrotor_interface/limiters.h>
#include <hector_quadrotor_interface/quadrotor_interface.h>
#include <hector_quadrotor_interface/helpers.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>
#include <Eigen/Dense>
#include <boost/thread/mutex.hpp>
#include <limits>

namespace motion_controllers{

using namespace hector_quadrotor_interface;

  class HGfilter{
  public:
    HGfilter(float d, float l):x(6),delta(d), lambda(l){
      Eigen::MatrixXd I = Eigen::MatrixXd::Identity(3,3);
      A << 0.9825*I,0.1637*I,-0.1637*I,0.655*I;
      B << 0.03275*I, 0.131*I;
      D << Eigen::MatrixXd::Zero(3,3), 0.2*I;
      x.setZero();
    }
    void update(double yaw, double pitch, double roll){
        u << roll, pitch, yaw;
        x = A*x+B*u;
        y = x + D*u;
      }
    void get(Eigen::Vector3d& omg, Eigen::Vector3d& domg){
      omg = y.bottomRows(3) / delta;
      domg = ((u - y.topRows(3))/delta - y.bottomRows(3) * lambda/delta)/delta;
    }
  private:
    Eigen::VectorXd x, y;
    Eigen::Matrix<double, 6, 6> A;
    Eigen::Matrix<double, 6, 3> B;
    Eigen::Matrix<double, 6, 3> D;
    Eigen::Vector3d u;
    double delta, lambda;
  };

  class AttitudeController : public controller_interface::Controller<hector_quadrotor_interface::QuadrotorInterface>
  {
  public:
    AttitudeController(){
    }

    ~AttitudeController(){
    }

    virtual bool init(hector_quadrotor_interface::QuadrotorInterface *interface, ros::NodeHandle &root_nh,
              ros::NodeHandle &controller_nh)
    {
      pose_ = interface->getPose();
      twist_ = interface->getTwist();
      accel_ = interface->getAccel();
      motor_status_ = interface->getMotorStatus();
      root_nh.param("/robot/mass", mass_, 1.2);
      root_nh.param("/robot/Ixx",inertia_[0],0.01);
      root_nh.param("/robot/Iyy",inertia_[1],0.01);
      root_nh.param("/robot/Izz",inertia_[2],0.01);

      attitude_input_ = interface->addInput<AttitudeCommandHandle>("attitude");
      heading_input_ = interface->addInput<HeadingCommandHandle>("heading");
      thrust_input_ = interface->addInput<ThrustCommandHandle>("thrust");
      wrench_output_ = interface->addOutput<WrenchCommandHandle>("wrench");

      attitude_limiter_.init(controller_nh);
      heading_limiter_.init(controller_nh, "yaw");
      controller_nh.param("kp", kp, 2.0);
      controller_nh.param("kd", kd, 15.0);
      filter = new HGfilter(20, 2);
      h = 1;

      return true;
    }

    void reset()
    {
      //pid_.roll.reset();
      //pid_.pitch.reset();
      //pid_.yawrate.reset();
      wrench_control_ = geometry_msgs::Wrench();
    }

    virtual void starting(const ros::Time &time)
    {
      reset();
      wrench_output_->start();
    }

    virtual void stopping(const ros::Time &time)
    {
      wrench_output_->stop();
    }

    virtual void update(const ros::Time &time, const ros::Duration &period)
    {
      boost::mutex::scoped_lock lock(command_mutex_);
      double heading_cmd;

      if (attitude_input_->connected() && attitude_input_->enabled())
      {
        attitude_command_ = attitude_input_->getCommand();
      }
      if (heading_input_->connected() && heading_input_->enabled())
      {
        heading_cmd = heading_input_->getCommand();
      }
      if (thrust_input_->connected() && thrust_input_->enabled())
      {
        thrust_command_ = thrust_input_->getCommand();
      }

      attitude_command_ = attitude_limiter_(attitude_command_);
      heading_cmd = heading_limiter_(heading_cmd);
      thrust_command_ = thrust_limiter_(thrust_command_);

      if (!motor_status_->motorStatus().running)
        return;

      geometry_msgs::Pose pose = pose_->pose();
      tf2::Quaternion qd, q;
      tf2::fromMsg(pose.orientation, q);
      double dp = (double)attitude_command_.pitch;
      double dr = (double)attitude_command_.roll;
      double dy = heading_cmd;
      qd.setRPY(dr, dp, dy);//setEuler(dp, dr, dy);
      tf2::Quaternion eq = qd.inverse();
      eq *= q;

      Eigen::Vector3d omg, domg;

      filter->update(dy, dp, dr);
      filter->get(omg, domg);

      tf2::Matrix3x3 Rq(eq);
      tf2::Vector3 wd_(omg(0), omg(1), omg(2));
      tf2::Vector3 wd = Rq.transpose() * wd_;
      if(h*eq.w() <= -0.1){
        if(eq.w() > 0)
          h = 1;
        else if(eq.w() < 0)
          h = -1;
      }
      geometry_msgs::Twist twist = twist_->twist();
      tf2::Vector3 w(twist.angular.x, twist.angular.y, twist.angular.z);
      tf2::Vector3 eomg = w - wd;
      tf2::Matrix3x3 J(getInertia());
      tf2::Vector3 u = -kp * tf2::Vector3(eq.x(),eq.y(),eq.z())*h - kd * eomg;
      tf2::Vector3 dwd(domg(0), domg(1), domg(2));
      u = u - getS(J*wd)*wd + J * Rq.transpose() * dwd;

      tf2::Matrix3x3 R(q);
      double yaw, pitch, roll;
      R.getEulerYPR(yaw, pitch, roll);
      wrench_control_.torque.x = u.x();//roll torque
      wrench_control_.torque.y = u.y();//pitch torque
      wrench_control_.torque.z = u.z();//yaw torque
      wrench_control_.force.x  = 0.0;
      wrench_control_.force.y  = 0.0;
      wrench_control_.force.z = cos(attitude_command_.roll) * cos(attitude_command_.pitch) / (cos(roll)*cos(pitch)) * thrust_command_.thrust;//uf

      // set wrench output
      wrench_output_->setCommand(wrench_control_);
    }

  private:
    tf2::Matrix3x3 getS(const tf2::Vector3& v){
      return tf2::Matrix3x3(0, -v[2], v[1],v[2], 0, -v[0],-v[1], v[0], 0);
    }
    tf2::Matrix3x3 getInertia(){
      return tf2::Matrix3x3(inertia_[0], 0, 0,0, inertia_[1], 0,0, 0, inertia_[2]);
    }
    PoseHandlePtr pose_;
    TwistHandlePtr twist_;
    AccelerationHandlePtr accel_;
    MotorStatusHandlePtr motor_status_;

    AttitudeCommandHandlePtr attitude_input_;
    HeadingCommandHandlePtr heading_input_;
    ThrustCommandHandlePtr thrust_input_;
    WrenchCommandHandlePtr wrench_output_;

    hector_uav_msgs::AttitudeCommand attitude_command_;
    hector_uav_msgs::ThrustCommand thrust_command_;
    geometry_msgs::Wrench wrench_control_;

    hector_quadrotor_interface::AttitudeCommandLimiter attitude_limiter_;
    hector_quadrotor_interface::FieldLimiter<double> heading_limiter_;
    hector_quadrotor_interface::ThrustCommandLimiter thrust_limiter_;

    HGfilter* filter;

    double mass_;
    double inertia_[3];
    double kp, kd;
    int h;

    boost::mutex command_mutex_;
  };

} // namespace motion_controllers

PLUGINLIB_EXPORT_CLASS(motion_controllers::AttitudeController, controller_interface::ControllerBase)