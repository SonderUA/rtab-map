#!/usr/bin/env python3
"""Run the six-axis Gazebo flight with visual odometry paused during a turn."""

from bisect import bisect_left
import math
import subprocess
import time

import numpy as np
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from std_srvs.srv import Empty
from tf.transformations import quaternion_matrix


def wait_until(condition, wall_seconds, description):
    deadline = time.monotonic() + wall_seconds
    while not condition():
        if rospy.is_shutdown() or time.monotonic() >= deadline:
            raise RuntimeError(description)
        time.sleep(0.02)


def stamp(message):
    return message.header.stamp.to_sec()


def transform(message):
    pose = message.pose.pose
    q = pose.orientation
    values = (q.x, q.y, q.z, q.w, pose.position.x, pose.position.y, pose.position.z)
    if not all(math.isfinite(value) for value in values) or sum(v * v for v in values[:4]) < 1e-6:
        raise RuntimeError("Invalid pose quaternion or position")
    matrix = quaternion_matrix(values[:4])
    matrix[:3, 3] = values[4:]
    return matrix


def compare_poses(fused, truth):
    truth_stamps = [stamp(sample) for sample in truth]
    if not truth_stamps:
        raise RuntimeError("Gazebo ground truth is missing")

    def nearest_truth(sample):
        index = bisect_left(truth_stamps, stamp(sample))
        candidates = (max(0, index - 1), min(len(truth) - 1, index))
        best = min(candidates, key=lambda i: abs(truth_stamps[i] - stamp(sample)))
        if abs(truth_stamps[best] - stamp(sample)) > 0.05:
            raise RuntimeError("Could not match the fused pose to ground truth within 50 ms")
        return transform(truth[best])

    # The filter's local frame starts at the first camera pose.
    alignment = transform(fused[0]) @ np.linalg.inv(nearest_truth(fused[0]))
    return alignment, nearest_truth


def run():
    if not rospy.get_param("/use_sim_time", False):
        raise RuntimeError("This test requires a fresh Gazebo simulation")
    if not rospy.get_param("/omnicopter/publish_ground_truth", False):
        raise RuntimeError("Start the test simulator with OMNI_GROUND_TRUTH=true")

    outage = float(rospy.get_param("~dropout_seconds", 2.0))
    if not math.isfinite(outage) or outage <= 0:
        raise RuntimeError("dropout_seconds must be finite and positive")

    services = ("/gazebo/unpause_physics", "/gazebo/pause_physics",
                "/rtabmap/pause_odom", "/rtabmap/resume_odom")
    for name in services:
        rospy.wait_for_service(name, timeout=15)
    unpause, pause, stop_vision, resume_vision = (
        rospy.ServiceProxy(name, Empty) for name in services)

    fused, truth, visual, armed = [], [], [], []

    def record_visual(sample):
        visual.append((rospy.get_time(), sample))

    subscribers = [
        rospy.Subscriber("/omnicopter/odometry/filtered", Odometry, fused.append, queue_size=100),
        rospy.Subscriber("/omnicopter/ground_truth/odometry", Odometry, truth.append, queue_size=100),
        rospy.Subscriber("/rtabmap/odom", Odometry, record_visual, queue_size=20),
        rospy.Subscriber("/toggle_topic", Bool, lambda msg: armed.append(msg.data), queue_size=10),
    ]
    flight = None
    unpause()
    try:
        wait_until(lambda: fused and truth and rospy.get_time() >= 1.0, 30, "Pose streams did not start")
        flight_start = rospy.get_time()
        if flight_start > 5.0:
            raise RuntimeError("Restart Gazebo before running the test")
        flight = subprocess.Popen(["rosrun", "omnicopter_slam", "flight_demo", "--six-dof"])

        # The third leg reverses roll, pitch, and yaw while moving and descending.
        wait_until(lambda: rospy.get_time() >= flight_start + 28.0 or flight.poll() is not None,
                   120, "The flight did not reach the reversal")
        if flight.poll() is not None:
            raise RuntimeError("The flight ended before the interruption")
        stop_vision()
        start = rospy.get_time()
        try:
            wait_until(lambda: rospy.get_time() >= start + outage, 30, "Clock stalled during the interruption")
        finally:
            resume_requested = rospy.get_time()
            resume_vision()
        end = rospy.get_time()
        print(f"Paused visual odometry for {end - start:.2f} simulated seconds at t={start:.2f}s", flush=True)

        wait_until(lambda: rospy.get_time() >= end + 1.0 or flight.poll() is not None,
                   30, "Clock stalled while waiting for visual recovery")
        fused_samples, truth_samples, visual_samples = list(fused), list(truth), list(visual)
        recovered = next((received for received, sample in visual_samples
                          if received >= end and stamp(sample) >= resume_requested
                          and sample.pose.covariance[0] < 9999.0
                          and sample.twist.covariance[0] < 9999.0), None)
        blackout = [sample for sample in fused_samples if start <= stamp(sample) <= end]
        interval = [start] + [stamp(sample) for sample in blackout] + [end]
        largest_gap = max(b - a for a, b in zip(interval, interval[1:]))
        alignment, nearest_truth = compare_poses(fused_samples, truth_samples)
        errors = []
        for sample in blackout:
            estimate = transform(sample)
            actual = alignment @ nearest_truth(sample)
            position = np.linalg.norm(estimate[:3, 3] - actual[:3, 3])
            relative = actual[:3, :3].T @ estimate[:3, :3]
            cosine = np.clip((np.trace(relative) - 1.0) / 2.0, -1.0, 1.0)
            errors.append((position, math.degrees(math.acos(cosine))))
        position_error = max((error[0] for error in errors), default=float("inf"))
        attitude_error = max((error[1] for error in errors), default=float("inf"))
        print(f"During blackout: {len(blackout)} fused poses, max gap {largest_gap:.3f}s, "
              f"peak error {position_error:.2f} m and {attitude_error:.1f}°", flush=True)
        if recovered is not None:
            print(f"Visual odometry recovered {recovered - end:.2f}s after resuming", flush=True)

        failures = []
        # Frames buffered during the pause can be delivered after resume.
        if any(start + 0.10 < received < resume_requested for received, _ in visual_samples):
            failures.append("Visual odometry was still publishing during the interruption")
        if largest_gap > 0.10:
            failures.append("Fused output gap exceeded 0.10 s")
        if position_error > 0.75 or attitude_error > 20.0:
            failures.append("Pose error exceeded 0.75 m or 20°")
        if recovered is None or recovered - end > 1.0:
            failures.append("Visual odometry did not recover within 1.0 s")
        if failures:
            raise RuntimeError("; ".join(failures))
        if flight.wait(timeout=180) != 0 or armed != [True, False]:
            raise RuntimeError("The flight did not finish and disarm")
        print("PASS: pose stayed within limits, visual odometry recovered, and the flight disarmed.", flush=True)
    finally:
        if flight is not None and flight.poll() is None:
            flight.terminate()
        pause()
        if flight is not None and flight.poll() is None:
            flight.wait(timeout=5)
        for subscriber in subscribers:
            subscriber.unregister()


if __name__ == "__main__":
    rospy.init_node("pose_dropout_test")
    try:
        run()
    except (RuntimeError, rospy.ROSException, rospy.ServiceException, subprocess.TimeoutExpired) as error:
        rospy.logerr("FAIL: %s", error)
        raise SystemExit(1)
