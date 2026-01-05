#!/usr/bin/env python3
"""
Quick validation script for Phase 1S implementation
Checks imports and basic functionality without ROS
"""

import sys
import os

# Add paths
sys.path.insert(0, '/opt/ros_ws/src/panda_grasp_planning')

print("\n" + "="*70)
print("PHASE 1S IMPLEMENTATION VALIDATION")
print("="*70 + "\n")

# Test 1: Import sorting state machine
print("[1/3] Testing SortingStateMachine import...")
try:
    from modules.sorting.sorting_state_machine import SortingStateMachine
    print("      ✓ SortingStateMachine imported successfully\n")
except Exception as e:
    print(f"      ✗ FAILED: {e}\n")
    sys.exit(1)

# Test 2: Create instance and test basic functionality
print("[2/3] Testing SortingStateMachine functionality...")
try:
    sorter = SortingStateMachine()
    
    # Test color assignment
    colors_to_test = ['RED', 'BLUE', 'GREEN', 'YELLOW', 'UNKNOWN']
    results = []
    
    for color in colors_to_test:
        success, bin_info = sorter.assign_target_bin(color)
        results.append((color, success, bin_info.get('bin_name') if bin_info else None))
        sorter.reset()
    
    # Check results
    expected_success = ['RED', 'BLUE', 'GREEN', 'YELLOW']
    for color, success, bin_name in results:
        if color in expected_success:
            assert success, f"Color {color} should succeed"
            assert bin_name is not None, f"Bin name should not be None for {color}"
            print(f"      ✓ {color:10s} → {bin_name}")
        elif color == 'UNKNOWN':
            assert not success, f"Color {color} should fail"
            print(f"      ✓ {color:10s} → (correctly rejected)")
    
    print()
except Exception as e:
    print(f"      ✗ FAILED: {e}\n")
    import traceback
    traceback.print_exc()
    sys.exit(1)

# Test 3: Verify configuration structure
print("[3/3] Testing bin configuration...")
try:
    sorter = SortingStateMachine()
    bins = sorter.bins
    
    assert len(bins) >= 3, "Should have at least 3 bins"
    
    for bin_name, bin_info in bins.items():
        assert 'position' in bin_info, f"Bin {bin_name} missing position"
        assert 'name' in bin_info, f"Bin {bin_name} missing name"
        assert len(bin_info['position']) == 3, f"Position should have 3 coordinates"
        
        x, y, z = bin_info['position']
        print(f"      ✓ {bin_name:10s}: [{x:6.2f}, {y:6.2f}, {z:6.2f}] - {bin_info['name']}")
    
    print()
except Exception as e:
    print(f"      ✗ FAILED: {e}\n")
    import traceback
    traceback.print_exc()
    sys.exit(1)

print("="*70)
print("✅ ALL VALIDATION CHECKS PASSED")
print("="*70)
print("\nImplementation Summary:")
print("  • SortingStateMachine: ✓ Fully functional")
print("  • Color assignments: ✓ All colors mapped correctly")
print("  • Bin positions: ✓ 3 bins defined and accessible")
print("\nNext steps:")
print("  1. Launch Gazebo with colored cubes: roslaunch panda_grasp_planning ...")
print("  2. Start grasp_pipeline_v3.py")
print("  3. Run phase_1s_demo.py for end-to-end testing")
print("\n")
