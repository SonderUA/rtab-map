#include <algorithm>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

class LocalizationInputs {
 public:
  LocalizationInputs() {
    visual_pub_ = nh_.advertise<nav_msgs::Odometry>("odometry/visual", 10);
    imu_raw_pub_ = nh_.advertise<sensor_msgs::Imu>("imu/data_raw", 50);
    imu_estimator_pub_ = nh_.advertise<sensor_msgs::Imu>("imu/estimator", 50);
    visual_sub_ = nh_.subscribe("/rtabmap/odom", 10, &LocalizationInputs::onVisualOdometry, this);
    imu_raw_sub_ = nh_.subscribe("imu/gazebo", 50, &LocalizationInputs::onRawImu, this);
    imu_attitude_sub_ = nh_.subscribe("imu/data", 50, &LocalizationInputs::onImuAttitude, this);
  }

 private:
  void onVisualOdometry(const nav_msgs::Odometry::ConstPtr& odometry) {
    // RTAB-Map publishes a null pose with covariance 9999 on tracking loss.
    constexpr double lost_covariance = 9999.0;
    if (odometry->header.stamp.isZero() || !(odometry->pose.covariance[0] < lost_covariance))
      return;
    // After tracking starts, a reset seed without a visual velocity can come
    // from our own TF prediction. Wait for a new registration before fusing it.
    if (!visual_stamp_.isZero() && odometry->header.stamp > visual_stamp_ &&
        !(odometry->twist.covariance[0] < lost_covariance))
      return;

    visual_stamp_ = odometry->header.stamp;
    nav_msgs::Odometry measurement = *odometry;
    // Visual twist is derived from poses and is noisier than the pose itself.
    // RTAB-Map uses a covariance of 9999 until a velocity is available.
    for (const auto axis : {0, 7, 14})
      measurement.twist.covariance[axis] = std::max(0.04, measurement.twist.covariance[axis]);
    visual_pub_.publish(measurement);
  }

  void onRawImu(const sensor_msgs::Imu::ConstPtr& sample) {
    sensor_msgs::Imu raw = *sample;
    // Derive attitude from gyro and accelerometer samples without Gazebo's quaternion.
    raw.orientation = geometry_msgs::Quaternion();
    raw.orientation_covariance.assign(0.0);
    raw.orientation_covariance[0] = -1.0;
    imu_raw_pub_.publish(raw);
  }

  void onImuAttitude(const sensor_msgs::Imu::ConstPtr& sample) {
    // Let visual odometry establish the EKF's reference frame.
    if (visual_stamp_.isZero() || sample->header.stamp < visual_stamp_)
      return;
    // Remove gravity with the IMU attitude at the sample timestamp.
    // The EKF still gets its absolute orientation corrections from vision.
    sensor_msgs::Imu measurement = *sample;
    for (const auto axis : {0, 4, 8})
      measurement.linear_acceleration_covariance[axis] += 0.01;
    imu_estimator_pub_.publish(measurement);
  }

  ros::NodeHandle nh_;
  ros::Publisher visual_pub_;
  ros::Publisher imu_raw_pub_;
  ros::Publisher imu_estimator_pub_;
  ros::Subscriber visual_sub_;
  ros::Subscriber imu_raw_sub_;
  ros::Subscriber imu_attitude_sub_;
  ros::Time visual_stamp_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "localization_inputs");
  LocalizationInputs inputs;
  ros::spin();
}
