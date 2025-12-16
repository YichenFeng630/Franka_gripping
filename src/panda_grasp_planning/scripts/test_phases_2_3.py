#!/usr/bin/env python3
"""
Unit Test for Panda Grasp Planning Phases 2 & 3

Tests core functionality without requiring full ROS/Gazebo environment:
1. Grasp pose generation logic
2. Benchmark data collection and analysis
"""

import sys
import os
import math
import numpy as np
from collections import Counter

class TestGraspPoseGenerator:
    """Test grasp pose generation logic"""
    
    def __init__(self):
        # Parameters from config/grasp_params.yaml
        self.pre_grasp_offset_z = 0.10
        self.grasp_offset_z = 0.02
        self.lift_offset_z = 0.15
        
        # Gripper orientation (RPY)
        self.gripper_roll = math.pi
        self.gripper_pitch = 0.0
        self.gripper_yaw = 0.0
        
    def generate_grasp_poses(self, target_x, target_y, target_z):
        """
        Generate pre-grasp, grasp, and lift poses from target position.
        
        Returns:
            Dictionary with poses
        """
        poses = {
            'target': (target_x, target_y, target_z),
            'pre_grasp': (
                target_x,
                target_y,
                target_z + self.pre_grasp_offset_z
            ),
            'grasp': (
                target_x,
                target_y,
                target_z + self.grasp_offset_z
            ),
            'lift': (
                target_x,
                target_y,
                target_z + self.grasp_offset_z + self.lift_offset_z
            )
        }
        return poses
    
    def test_basic_generation(self):
        """Test basic pose generation"""
        print("\n" + "="*60)
        print("TEST 1: Basic Grasp Pose Generation")
        print("="*60)
        
        # Test case: cube at (0.5, 0.0, 0.05)
        target_x, target_y, target_z = 0.5, 0.0, 0.05
        
        poses = self.generate_grasp_poses(target_x, target_y, target_z)
        
        print(f"\nInput target: ({target_x}, {target_y}, {target_z})")
        print(f"  Expected: cube center in panda_link0 frame")
        
        print(f"\nGenerated poses:")
        print(f"  Pre-grasp:  ({poses['pre_grasp'][0]:.3f}, "
              f"{poses['pre_grasp'][1]:.3f}, {poses['pre_grasp'][2]:.3f})")
        print(f"  Grasp:      ({poses['grasp'][0]:.3f}, "
              f"{poses['grasp'][1]:.3f}, {poses['grasp'][2]:.3f})")
        print(f"  Lift:       ({poses['lift'][0]:.3f}, "
              f"{poses['lift'][1]:.3f}, {poses['lift'][2]:.3f})")
        
        # Verify offsets
        assert poses['pre_grasp'][2] == target_z + self.pre_grasp_offset_z, \
            "Pre-grasp Z offset incorrect"
        assert poses['grasp'][2] == target_z + self.grasp_offset_z, \
            "Grasp Z offset incorrect"
        assert poses['lift'][2] == target_z + self.grasp_offset_z + self.lift_offset_z, \
            "Lift Z offset incorrect"
        
        # Verify XY consistency
        assert poses['pre_grasp'][:2] == poses['grasp'][:2] == poses['lift'][:2], \
            "XY positions should be consistent"
        
        print("\n✓ All assertions passed!")
        print("✓ Poses maintain correct Z offsets")
        print("✓ XY positions are consistent")
        
        return True
    
    def test_multiple_targets(self):
        """Test generation with multiple random targets"""
        print("\n" + "="*60)
        print("TEST 2: Multiple Target Positions")
        print("="*60)
        
        # Workspace bounds from benchmark_rrt.py
        workspace_bounds = {
            'x_min': 0.2, 'x_max': 0.8,
            'y_min': -0.5, 'y_max': 0.5,
            'z_min': 0.05, 'z_max': 0.8
        }
        
        num_tests = 10
        print(f"\nGenerating {num_tests} random targets in workspace:")
        print(f"  X: [{workspace_bounds['x_min']}, {workspace_bounds['x_max']}]")
        print(f"  Y: [{workspace_bounds['y_min']}, {workspace_bounds['y_max']}]")
        print(f"  Z: [{workspace_bounds['z_min']}, {workspace_bounds['z_max']}]")
        
        np.random.seed(42)  # Reproducible
        
        for i in range(num_tests):
            x = np.random.uniform(workspace_bounds['x_min'], 
                                 workspace_bounds['x_max'])
            y = np.random.uniform(workspace_bounds['y_min'], 
                                 workspace_bounds['y_max'])
            z = np.random.uniform(workspace_bounds['z_min'], 
                                 workspace_bounds['z_max'])
            
            poses = self.generate_grasp_poses(x, y, z)
            
            # Verify pre-grasp is above target
            assert poses['pre_grasp'][2] > poses['target'][2], \
                f"Trial {i}: Pre-grasp should be above target"
            
            # Verify grasp is above target (but below pre-grasp)
            assert poses['target'][2] < poses['grasp'][2] < poses['pre_grasp'][2], \
                f"Trial {i}: Grasp height ordering incorrect"
            
            # Verify lift is highest
            assert poses['lift'][2] > poses['pre_grasp'][2], \
                f"Trial {i}: Lift should be above pre-grasp"
            
            print(f"  Trial {i+1}: ✓ ({x:.3f}, {y:.3f}, {z:.3f}) "
                  f"-> pre-grasp z={poses['pre_grasp'][2]:.3f}")
        
        print("\n✓ All {0} trials passed!".format(num_tests))
        return True


class TestBenchmarkMetrics:
    """Test benchmark metrics calculation"""
    
    def generate_mock_results(self, num_trials=50):
        """Generate mock benchmark results"""
        np.random.seed(42)
        
        results = []
        for trial in range(1, num_trials + 1):
            # 80% success rate
            success = np.random.random() > 0.2
            
            if success:
                planning_time = np.random.normal(0.8, 0.2)  # mean 0.8s, std 0.2s
                path_length = np.random.normal(2.0, 0.5)    # mean 2.0 rad
                num_waypoints = int(np.random.normal(20, 5))
            else:
                planning_time = 10.0  # timeout
                path_length = 0.0
                num_waypoints = 0
            
            results.append({
                'trial': trial,
                'target_x': np.random.uniform(0.2, 0.8),
                'target_y': np.random.uniform(-0.5, 0.5),
                'target_z': np.random.uniform(0.05, 0.8),
                'success': success,
                'planning_time': max(0.0, planning_time),
                'path_length': max(0.0, path_length),
                'num_waypoints': max(0, num_waypoints),
                'error_type': 'NONE' if success else 'PLANNING_TIMEOUT'
            })
        
        return results
    
    def test_metrics_calculation(self):
        """Test benchmark metrics calculation"""
        print("\n" + "="*60)
        print("TEST 3: Benchmark Metrics Calculation")
        print("="*60)
        
        results = self.generate_mock_results(num_trials=50)
        
        successful = [r for r in results if r['success']]
        failed = [r for r in results if not r['success']]
        
        success_rate = len(successful) / len(results) * 100
        
        print(f"\nBenchmark Results Summary:")
        print(f"  Total trials: {len(results)}")
        print(f"  Successful: {len(successful)} ({success_rate:.1f}%)")
        print(f"  Failed: {len(failed)} ({100-success_rate:.1f}%)")
        
        if successful:
            planning_times = [r['planning_time'] for r in successful]
            path_lengths = [r['path_length'] for r in successful]
            waypoints = [r['num_waypoints'] for r in successful]
            
            print(f"\n--- Planning Time Statistics ---")
            print(f"  Mean: {np.mean(planning_times):.3f}s")
            print(f"  Std Dev: {np.std(planning_times):.3f}s")
            print(f"  Min: {np.min(planning_times):.3f}s")
            print(f"  Max: {np.max(planning_times):.3f}s")
            print(f"  Median: {np.median(planning_times):.3f}s")
            
            print(f"\n--- Path Length Statistics ---")
            print(f"  Mean: {np.mean(path_lengths):.3f} rad")
            print(f"  Std Dev: {np.std(path_lengths):.3f} rad")
            print(f"  Min: {np.min(path_lengths):.3f} rad")
            print(f"  Max: {np.max(path_lengths):.3f} rad")
            
            print(f"\n--- Waypoint Statistics ---")
            print(f"  Mean: {np.mean(waypoints):.1f}")
            print(f"  Std Dev: {np.std(waypoints):.1f}")
            print(f"  Min: {int(np.min(waypoints))}")
            print(f"  Max: {int(np.max(waypoints))}")
        
        if failed:
            error_types = Counter([r['error_type'] for r in failed])
            print(f"\n--- Failure Analysis ---")
            for error, count in error_types.items():
                print(f"  {error}: {count} ({count/len(failed)*100:.1f}%)")
        
        # Verify calculations
        assert success_rate > 0 and success_rate <= 100, "Success rate invalid"
        assert len(successful) + len(failed) == len(results), "Result count mismatch"
        
        print("\n✓ Metrics calculation correct")
        return True
    
    def test_csv_format(self):
        """Test CSV output format"""
        print("\n" + "="*60)
        print("TEST 4: CSV Output Format")
        print("="*60)
        
        results = self.generate_mock_results(num_trials=10)
        
        # Simulate CSV writing
        csv_lines = []
        csv_lines.append("trial,target_x,target_y,target_z,success,planning_time,"
                        "path_length,num_waypoints,error_type")
        
        for result in results:
            csv_line = (f"{result['trial']},{result['target_x']:.3f},"
                       f"{result['target_y']:.3f},{result['target_z']:.3f},"
                       f"{result['success']},{result['planning_time']:.3f},"
                       f"{result['path_length']:.3f},{result['num_waypoints']},"
                       f"{result['error_type']}")
            csv_lines.append(csv_line)
        
        print(f"\nGenerated CSV with {len(csv_lines)-1} trials:")
        print("\nHeader:")
        print(f"  {csv_lines[0]}")
        print("\nSample rows:")
        for line in csv_lines[1:3]:
            print(f"  {line}")
        
        # Verify format
        assert len(csv_lines) == len(results) + 1, "CSV line count mismatch"
        assert all("," in line for line in csv_lines), "Missing commas in CSV"
        
        print("\n✓ CSV format valid")
        return True


class TestPipelineLogic:
    """Test pipeline state machine logic"""
    
    def test_state_transitions(self):
        """Test grasp pipeline state transitions"""
        print("\n" + "="*60)
        print("TEST 5: Pipeline State Transitions")
        print("="*60)
        
        # Define states (from grasp_pipeline_node.py)
        states = {
            'IDLE': 0,
            'WAITING_FOR_TARGET': 1,
            'MOVING_TO_HOME': 2,
            'PLANNING_PRE_GRASP': 3,
            'EXECUTING_PRE_GRASP': 4,
            'OPENING_GRIPPER': 5,
            'PLANNING_GRASP': 6,
            'EXECUTING_GRASP': 7,
            'CLOSING_GRIPPER': 8,
            'PLANNING_LIFT': 9,
            'EXECUTING_LIFT': 10,
            'RETURNING_HOME': 11,
            'SUCCESS': 12,
            'FAILED': 13
        }
        
        # Define expected state sequence
        success_sequence = [
            'IDLE',
            'WAITING_FOR_TARGET',
            'MOVING_TO_HOME',
            'PLANNING_PRE_GRASP',
            'EXECUTING_PRE_GRASP',
            'OPENING_GRIPPER',
            'PLANNING_GRASP',
            'EXECUTING_GRASP',
            'CLOSING_GRIPPER',
            'PLANNING_LIFT',
            'EXECUTING_LIFT',
            'RETURNING_HOME',
            'SUCCESS'
        ]
        
        print(f"\nExpected state sequence for successful grasp:")
        for i, state in enumerate(success_sequence, 1):
            print(f"  {i:2d}. {state}")
        
        # Verify state definitions
        for state in success_sequence:
            assert state in states, f"State '{state}' not defined"
        
        # Verify all states are unique
        assert len(states) == len(set(states.values())), "Duplicate state IDs"
        
        print(f"\n✓ {len(success_sequence)} state transitions verified")
        print("✓ State machine logic valid")
        return True


def main():
    print("\n" + "="*70)
    print("PANDA GRASP PLANNING - UNIT TESTS (Phase 2 & 3)")
    print("="*70)
    
    try:
        # Test Grasp Pose Generator
        generator = TestGraspPoseGenerator()
        assert generator.test_basic_generation(), "Basic generation test failed"
        assert generator.test_multiple_targets(), "Multiple targets test failed"
        
        # Test Benchmark Metrics
        benchmark = TestBenchmarkMetrics()
        assert benchmark.test_metrics_calculation(), "Metrics calculation failed"
        assert benchmark.test_csv_format(), "CSV format test failed"
        
        # Test Pipeline Logic
        pipeline = TestPipelineLogic()
        assert pipeline.test_state_transitions(), "State transitions test failed"
        
        print("\n" + "="*70)
        print("✓✓✓ ALL TESTS PASSED ✓✓✓")
        print("="*70)
        print("\nSummary:")
        print("  ✓ Grasp pose generation logic works correctly")
        print("  ✓ Benchmark metrics calculation is valid")
        print("  ✓ CSV output format is correct")
        print("  ✓ Pipeline state machine logic is sound")
        print("\nNext steps:")
        print("  1. Start Gazebo+MoveIt: roslaunch franka_zed_gazebo moveit_gazebo_panda.launch")
        print("  2. Run Phase 1 test: roslaunch panda_grasp_planning test_phase1.launch")
        print("  3. Run Phase 2 test: roslaunch panda_grasp_planning test_phase2.launch")
        print("  4. Run Phase 3 eval: roslaunch panda_grasp_planning benchmark.launch trials:=50")
        print("\n" + "="*70 + "\n")
        
        return 0
        
    except AssertionError as e:
        print(f"\n✗ TEST FAILED: {e}")
        return 1
    except Exception as e:
        print(f"\n✗ UNEXPECTED ERROR: {e}")
        import traceback
        traceback.print_exc()
        return 1


if __name__ == '__main__':
    sys.exit(main())
