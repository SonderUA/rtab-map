#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <omni_firmware/FullPose.h>
#include <maneuver/ArmDisarm.h>
#include <maneuver/LiftOff.h>
#include <maneuver/GotoPoint.h>
#include <maneuver/Goto6DPoint.h>
#include <maneuver/RotateTo.h>
#include <maneuver/Land.h>

#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

// The maneuver node plans each leg from the current estimated pose.
class FlightDemo {
 public:
  FlightDemo() {
    pose_sub_ = nh_.subscribe("/pose_full", 1, &FlightDemo::onPose, this);
  }

  void run(bool six_dof) {
    bool simulation_time = false;
    nh_.getParam("/use_sim_time", simulation_time);
    if (!simulation_time)
      throw std::runtime_error("This demonstration requires the Gazebo simulation.");

    std_srvs::Empty empty;
    invoke("/gazebo/unpause_physics", empty);
    const auto deadline = ros::WallTime::now() + ros::WallDuration(60);
    while (ros::ok()) {
      {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        if (have_pose_) {
          if (pose_.pose.position.z > 0.5)
            throw std::runtime_error("Start the demo from a fresh simulation on the ground.");
          break;
        }
      }
      if (ros::WallTime::now() > deadline)
        throw std::runtime_error("No fused camera/IMU pose received.");
      ros::WallDuration(0.05).sleep();
    }
    if (!ros::ok()) return;

    maneuver::ArmDisarm arm;
    invoke("/ArmDisarm", arm);
    accepted(arm.response.success, "ArmDisarm");
    ROS_INFO("Taking off to 1.2 metres.");
    maneuver::LiftOff takeoff;
    takeoff.request.height = 1.2;
    takeoff.request.duration = 8.0;
    invoke("/lift_off", takeoff);
    accepted(takeoff.response.status, "lift_off");
    waitSim(11);

    if (six_dof)
      flySixDofRoute();
    else
      flyObservationRoute();

    ROS_INFO("Landing.");
    maneuver::Land land;
    land.request.height_1 = 0.5;
    land.request.duration_1 = 4;
    land.request.height_2 = 0.05;
    land.request.duration_2 = 4;
    invoke("/land", land);
    accepted(land.response.status, "land");
    waitSim(11);
    invoke("/ArmDisarm", arm);
    accepted(arm.response.success, "ArmDisarm");
    invoke("/gazebo/pause_physics", empty);
    ROS_INFO("Demonstration finished. Physics is paused for viewing the map.");
  }

 private:
  template <typename Service>
  void invoke(const std::string& name, Service& service) {
    auto client = nh_.serviceClient<Service>(name);
    if (!client.waitForExistence(ros::Duration(60)) || !client.call(service))
      throw std::runtime_error("Service unavailable or failed: " + name);
  }

  void accepted(bool status, const std::string& command) {
    if (!status) throw std::runtime_error("Command rejected: " + command);
  }

  void waitSim(double seconds) {
    const auto start = ros::Time::now();
    const auto deadline = ros::WallTime::now() + ros::WallDuration(240);
    while (ros::ok() && (ros::Time::now() - start).toSec() < seconds) {
      if (ros::WallTime::now() > deadline)
        throw std::runtime_error("Simulation clock stalled; restart the simulation.");
      ros::WallDuration(0.05).sleep();
    }
    if (!ros::ok()) throw std::runtime_error("ROS stopped.");
  }

  void flyObservationRoute() {
    goTo(0, -1);
    goTo(1, -1);
    const double pi = 3.141592653589793;
    ROS_INFO("Turning to observe the surroundings.");
    for (double heading : {pi / 2, pi, -pi / 2, 0.0}) {
      maneuver::RotateTo turn;
      turn.request.roll = 0;
      turn.request.pitch = 0;
      turn.request.yaw = heading;
      turn.request.duration = 7.0;
      invoke("/rotate_to", turn);
      accepted(turn.response.status, "rotate_to");
      waitSim(10);
    }
    goTo(0, 0);
  }

  void flySixDofRoute() {
    struct Waypoint {
      float x, y, z;
      float roll, pitch, yaw;
      float seconds;
    };
    const Waypoint route[] = {
        { 0.65f, -0.85f, 1.65f,  0.26f, -0.20f,  0.60f, 5.5f},
        {-0.80f, -1.15f, 2.05f, -0.33f,  0.27f, -0.75f, 6.0f},
        {-1.10f,  0.60f, 1.00f,  0.35f, -0.29f,  0.95f, 6.0f},
        { 0.65f,  0.95f, 1.85f, -0.28f,  0.25f, -0.95f, 6.0f},
        { 0.30f, -0.45f, 0.90f,  0.20f, -0.18f,  0.55f, 5.0f},
        { 0.00f,  0.00f, 1.20f,  0.00f,  0.00f,  0.00f, 5.0f},
    };

    ROS_INFO("Starting the six-degree-of-freedom stress test.");
    for (const auto& waypoint : route) {
      ROS_INFO("Target XYZ (%.2f, %.2f, %.2f) m, RPY (%.2f, %.2f, %.2f) rad in %.1f s.",
               waypoint.x, waypoint.y, waypoint.z, waypoint.roll, waypoint.pitch,
               waypoint.yaw, waypoint.seconds);
      maneuver::Goto6DPoint target;
      target.request.x = waypoint.x;
      target.request.y = waypoint.y;
      target.request.z = waypoint.z;
      target.request.roll = waypoint.roll;
      target.request.pitch = waypoint.pitch;
      target.request.yaw = waypoint.yaw;
      target.request.duration = waypoint.seconds;
      invoke("/go_to_6D", target);
      accepted(target.response.status, "go_to_6D");
      waitSim(waypoint.seconds + 1.5);
    }
  }

  void goTo(double x, double y) {
    ROS_INFO("Moving to (%.1f, %.1f, 1.2) in the visual odometry frame.", x, y);
    maneuver::GotoPoint target;
    target.request.x = x;
    target.request.y = y;
    target.request.z = 1.2;
    target.request.yaw = 0;
    target.request.duration = 8;
    invoke("/go_to", target);
    accepted(target.response.status, "go_to");
    waitSim(11);
  }

  void onPose(const omni_firmware::FullPose::ConstPtr& pose) {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    pose_ = *pose;
    have_pose_ = true;
  }

  ros::NodeHandle nh_;
  ros::Subscriber pose_sub_;
  std::mutex pose_mutex_;
  omni_firmware::FullPose pose_;
  bool have_pose_ = false;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "flight_demo");
  std::vector<std::string> args;
  ros::removeROSArgs(argc, argv, args);
  const bool six_dof = args.size() == 2 && args[1] == "--six-dof";
  if (args.size() > 1 && !six_dof) {
    ROS_ERROR("Usage: flight_demo [--six-dof]");
    return 1;
  }
  try {
    FlightDemo demo;
    // Stop callback threads before destroying the subscriber's pose storage,
    // including when a service fails and the scope exits through an exception.
    ros::AsyncSpinner spinner(1);
    spinner.start();
    demo.run(six_dof);
    return 0;
  } catch (const std::exception& error) {
    ROS_ERROR("%s", error.what());
    return 1;
  }
}
