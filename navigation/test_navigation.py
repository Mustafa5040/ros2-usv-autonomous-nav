#!/usr/bin/env python3
# ---------------------------------------------------------------------------
#  ERTUĞRUL USV - POOL TEST RUNNER
#  This script is designed to safely and interactively test the Nav2 and
#  controller architecture step-by-step after launching the vessel in water.
# ---------------------------------------------------------------------------

import rclpy
import time
import threading
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from navigation_controller import NavigationController

def wait_for_user_confirmation(step_name):
    print(f"\n{'='*60}")
    print(f" [PREPARE] NEXT TEST: {step_name}")
    print(f"{'='*60}")
    input(" -> Press [ENTER] to start (CTRL+C to cancel)... ")

def wait_for_task_completion(navigator, timeout_sec=60.0):
    start_time = time.time()
    while not navigator.is_current_nav_task_completed():
        if time.time() - start_time > timeout_sec:
            print("\n[NAV2] Test timeout reached! Triggering emergency stop...")
            navigator.stop()
            return False
        time.sleep(0.5)
    return True

def run_pool_tests(navigator, node):
    try:
        print("\n[INFO] Navigation Test Started. Waiting for TF and Odometry...")
        time.sleep(3.0) 

        current_pose = navigator.get_vehicle_position()
        if current_pose is None:
            print("[ERROR] Could not acquire TF and Odometry! Canceling test...")
            return

        start_x = current_pose.pose.position.x
        start_y = current_pose.pose.position.y
        print(f"[SUCCESS] Vehicle position acquired -> X: {start_x:.2f}, Y: {start_y:.2f}")

        # -------------------------------------------------------------------
        # TEST 1: LINEAR MOVEMENT (HELLO WORLD)
        # -------------------------------------------------------------------
        wait_for_user_confirmation("1.5 Meters Linear Movement")
        print("[RUNNING] Vehicle moving 1.5 meters forward...")
        navigator.move_linear(1.5)
        
        if wait_for_task_completion(navigator, timeout_sec=20.0):
            print("[SUCCESS] Linear Movement Completed!")
        else:
            print("[FAILED] Linear Movement Failed!")

        # -------------------------------------------------------------------
        # TEST 2: FULL TURN AROUND A POINT (FOLLOW WAYPOINTS)
        # -------------------------------------------------------------------
        wait_for_user_confirmation("Full Turn Around a Point (followWaypoints)")
        
        # We assume 2 meters ahead of the current position as the center of the virtual buoy
        center_pose = navigator.get_vehicle_position()
        center_x = center_pose.pose.position.x + 2.0
        center_y = center_pose.pose.position.y
        
        print(f"[RUNNING] Circling around center (X: {center_x:.2f}, Y: {center_y:.2f}) with a 2m radius...")
        navigator.full_turn_around_point(center_x, center_y, radius=2.0)
        
        if wait_for_task_completion(navigator, timeout_sec=60.0):
            print("[SUCCESS] Full Turn Around Point Completed!")
        else:
            print("[FAILED] Full Turn Around Point Failed!")

        # -------------------------------------------------------------------
        # TEST 3: EMERGENCY STOP AND BRAKE TEST
        # -------------------------------------------------------------------
        wait_for_user_confirmation("Emergency Stop (E-Stop) Test")
        print("[RUNNING] Driving vehicle forward; E-STOP will be triggered in 3 seconds...")
        navigator.move_linear(5.0)
        time.sleep(3.0)
        
        print("\n[EMERGENCY STOP] TRIGGERED!")
        navigator.stop()
        print("[SUCCESS] Vehicle stopped and zero velocity command published.")

        print(f"\n{'='*60}")
        print(" [CONGRATULATIONS] ALL POOL TEST SCENARIOS COMPLETED! 🚀")
        print(f"{'='*60}\n")

    except Exception as e:
        print(f"\n[UNEXPECTED ERROR] An error occurred during testing: {e}")
        navigator.stop()

def main():
    rclpy.init()
    node = rclpy.create_node("pool_test_runner_node")

    tf_buffer = Buffer()
    TransformListener(tf_buffer, node)

    # global_frame is set to 'odom' since we are operating with odometry/GPS
    navigator = NavigationController(
        node, tf_buffer, global_frame="odom", robot_frame="base_link"
    )

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)

    # Launching ROS 2 spin loop in a background thread to prevent input() from blocking
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        run_pool_tests(navigator, node)
    except KeyboardInterrupt:
        print("\n[USER ABORT] Test manually terminated. Stopping vehicle...")
        navigator.stop()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()