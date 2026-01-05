#!/usr/bin/env python3
"""
Sorting State Machine for Phase 1S

Maps object colors to sorting bins and manages bin assignment logic.
"""

import rospy
from enum import Enum
from typing import Dict, Tuple, List


class SortingColor(Enum):
    """Supported cube colors"""
    RED = 'RED'
    BLUE = 'BLUE'
    GREEN = 'GREEN'
    YELLOW = 'YELLOW'
    UNKNOWN = 'UNKNOWN'


class SortingStateMachine:
    """
    Manages color-based sorting logic.
    
    Maps cube colors to target bin locations.
    Bins are positioned to avoid collisions and separate from each other.
    """
    
    def __init__(self, config_dict=None):
        """
        Initialize sorting state machine with bin locations.
        
        Args:
            config_dict: Optional configuration dictionary with bin positions.
                        If None, uses default configuration.
        """
        self.current_object_color = None
        self.current_target_bin = None
        
        # Define bin positions (non-overlapping, separated)
        # Each bin is a dictionary with position and metadata
        if config_dict is None:
            self.bins = self._get_default_bins()
        else:
            self.bins = config_dict
        
        # Map colors to bin names
        self.color_to_bin_map = {
            'RED': 'BIN_1',
            'BLUE': 'BIN_2',
            'GREEN': 'BIN_3',
            'YELLOW': 'BIN_1'  # Multiple colors can map to same bin if needed
        }
        
        rospy.loginfo("[SortingStateMachine] Initialized with {} bins".format(len(self.bins)))
        self._log_bin_configuration()
    
    def _get_default_bins(self) -> Dict[str, Dict]:
        """
        Define default bin positions in workspace.
        
        Constraint:
        - Bins must not collide with table, robot, or each other
        - Bins positioned behind/beside the table
        
        Workspace reference frame: panda_link0 (robot base)
        Table center: (0.5, 0.0, 0.0)
        Table size: 0.6m (X) x 0.75m (Y)
        Table top: z ≈ 0.1m
        
        Returns:
            Dictionary mapping bin names to positions and properties
        """
        return {
            'BIN_1': {
                'name': 'RED Bin',
                'position': [0.1, -0.35, 0.05],  # Left-rear of table
                'color_label': 'RED',
                'capacity': 10,
                'description': 'Bin for RED cubes'
            },
            'BIN_2': {
                'name': 'BLUE Bin',
                'position': [0.1, 0.0, 0.05],    # Center-rear of table
                'color_label': 'BLUE',
                'capacity': 10,
                'description': 'Bin for BLUE cubes'
            },
            'BIN_3': {
                'name': 'GREEN Bin',
                'position': [0.1, 0.35, 0.05],   # Right-rear of table
                'color_label': 'GREEN',
                'capacity': 10,
                'description': 'Bin for GREEN cubes'
            }
        }
    
    def assign_target_bin(self, object_color: str) -> Tuple[bool, Dict]:
        """
        Assign target bin based on object color.
        
        Args:
            object_color: Color of the grasped object ('RED', 'BLUE', 'GREEN', 'YELLOW')
        
        Returns:
            Tuple of (success: bool, bin_info: dict)
            - success: True if bin assignment succeeded
            - bin_info: Dictionary containing:
              - 'bin_name': Name of target bin
              - 'position': [x, y, z] target position
              - 'description': Human-readable description
              - 'color_label': Expected color in bin
        """
        if object_color not in self.color_to_bin_map:
            rospy.logwarn(f"[SortingStateMachine] Unknown color: {object_color}. Cannot assign bin.")
            return False, {}
        
        # Get bin name from color
        bin_name = self.color_to_bin_map[object_color]
        
        # Get bin information
        if bin_name not in self.bins:
            rospy.logerr(f"[SortingStateMachine] Bin '{bin_name}' not found in configuration!")
            return False, {}
        
        bin_info = self.bins[bin_name].copy()
        bin_info['bin_name'] = bin_name
        
        self.current_object_color = object_color
        self.current_target_bin = bin_name
        
        rospy.loginfo(
            f"[SortingStateMachine] ✓ Assigned {object_color} cube to {bin_name} "
            f"at position [{bin_info['position'][0]:.2f}, {bin_info['position'][1]:.2f}, {bin_info['position'][2]:.2f}]"
        )
        
        return True, bin_info
    
    def get_bin_position(self, bin_name: str) -> Tuple[bool, List[float]]:
        """
        Get 3D position of a specific bin.
        
        Args:
            bin_name: Name of the bin (e.g., 'BIN_1', 'BIN_2')
        
        Returns:
            Tuple of (success: bool, position: [x, y, z])
        """
        if bin_name not in self.bins:
            rospy.logerr(f"[SortingStateMachine] Bin '{bin_name}' not found!")
            return False, None
        
        position = self.bins[bin_name]['position']
        return True, position
    
    def get_current_target_bin(self) -> Tuple[bool, Dict]:
        """
        Get information about current target bin (after assignment).
        
        Returns:
            Tuple of (success: bool, bin_info: dict)
        """
        if self.current_target_bin is None:
            return False, {}
        
        bin_info = self.bins[self.current_target_bin].copy()
        bin_info['bin_name'] = self.current_target_bin
        
        return True, bin_info
    
    def reset(self):
        """Reset sorting state machine for next object."""
        self.current_object_color = None
        self.current_target_bin = None
        rospy.loginfo("[SortingStateMachine] Reset state")
    
    def _log_bin_configuration(self):
        """Log bin configuration for debugging."""
        rospy.loginfo("=" * 70)
        rospy.loginfo("[SortingStateMachine] BIN CONFIGURATION")
        rospy.loginfo("=" * 70)
        
        for bin_name, bin_info in self.bins.items():
            pos = bin_info['position']
            rospy.loginfo(
                f"  {bin_name:10s} {bin_info['name']:20s} "
                f"Position: [{pos[0]:+.2f}, {pos[1]:+.2f}, {pos[2]:.2f}]"
            )
        
        rospy.loginfo("=" * 70)
        rospy.loginfo("[SortingStateMachine] COLOR-TO-BIN MAPPING")
        rospy.loginfo("=" * 70)
        
        for color, bin_name in self.color_to_bin_map.items():
            rospy.loginfo(f"  {color:10s} → {bin_name}")
        
        rospy.loginfo("=" * 70)


def load_sorting_config(config_file=None) -> Dict:
    """
    Load sorting configuration from file (for future extensibility).
    
    Args:
        config_file: Path to YAML config file (optional)
    
    Returns:
        Configuration dictionary
    """
    if config_file is None:
        return None
    
    try:
        import yaml
        with open(config_file, 'r') as f:
            config = yaml.safe_load(f)
        rospy.loginfo(f"Loaded sorting config from {config_file}")
        return config
    except Exception as e:
        rospy.logwarn(f"Failed to load sorting config: {e}. Using defaults.")
        return None


if __name__ == '__main__':
    """Test sorting state machine"""
    rospy.init_node('sorting_state_machine_test')
    
    print("\n" + "=" * 70)
    print("SORTING STATE MACHINE TEST")
    print("=" * 70 + "\n")
    
    # Initialize
    sorter = SortingStateMachine()
    
    # Test assignments
    test_colors = ['RED', 'BLUE', 'GREEN', 'YELLOW', 'UNKNOWN']
    
    for color in test_colors:
        success, bin_info = sorter.assign_target_bin(color)
        if success:
            print(f"✓ {color:10s} → {bin_info['bin_name']} at {bin_info['position']}")
        else:
            print(f"✗ {color:10s} → Assignment failed")
        sorter.reset()
    
    print("\n" + "=" * 70 + "\n")
