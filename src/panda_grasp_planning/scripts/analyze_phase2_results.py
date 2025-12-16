#!/usr/bin/env python3
"""
Phase 2: Grasp Pipeline Results Analysis Tool
Analyzes grasp execution results and generates visualizations.

Usage:
    python3 analyze_phase2_results.py phase2_grasp_results.csv --plot
"""

import sys
import argparse
import csv
import numpy as np
from collections import Counter
import os


class Phase2Analyzer:
    def __init__(self, csv_file):
        self.csv_file = csv_file
        self.results = []
        self.successful = []
        self.failed = []
        self.load_data()
    
    def load_data(self):
        """Load data from CSV file"""
        print(f"Loading data from: {self.csv_file}")
        
        try:
            with open(self.csv_file, 'r') as f:
                reader = csv.DictReader(f)
                for row in reader:
                    row['trial'] = int(row['trial'])
                    row['target_x'] = float(row['target_x'])
                    row['target_y'] = float(row['target_y'])
                    row['target_z'] = float(row['target_z'])
                    row['success'] = row['success'].lower() == 'true'
                    row['total_time'] = float(row['total_time'])
                    row['avg_planning_time'] = float(row['avg_planning_time'])
                    row['avg_execution_time'] = float(row['avg_execution_time'])
                    row['num_stages'] = int(row['num_stages'])
                    
                    self.results.append(row)
                    if row['success']:
                        self.successful.append(row)
                    else:
                        self.failed.append(row)
            
            print(f"✓ Loaded {len(self.results)} trials")
            print(f"  - Successful: {len(self.successful)}")
            print(f"  - Failed: {len(self.failed)}\n")
            
        except FileNotFoundError:
            print(f"✗ Error: File not found: {self.csv_file}")
            sys.exit(1)
        except Exception as e:
            print(f"✗ Error loading data: {e}")
            sys.exit(1)
    
    def compute_statistics(self):
        """Compute detailed statistics"""
        if not self.results:
            print("No data to analyze")
            return
        
        total = len(self.results)
        successes = len(self.successful)
        failures = len(self.failed)
        success_rate = (successes / total) * 100
        
        print("="*70)
        print("PHASE 2 GRASP PIPELINE ANALYSIS")
        print("="*70)
        print(f"\nDataset: {self.csv_file}")
        print(f"Total trials: {total}")
        print(f"Successful: {successes} ({success_rate:.1f}%)")
        print(f"Failed: {failures} ({100-success_rate:.1f}%)")
        
        if successes > 0:
            print("\n" + "-"*70)
            print("SUCCESS METRICS")
            print("-"*70)
            
            total_times = [r['total_time'] for r in self.successful]
            planning_times = [r['avg_planning_time'] for r in self.successful]
            execution_times = [r['avg_execution_time'] for r in self.successful]
            
            print(f"Total time per grasp:")
            print(f"  Mean:   {np.mean(total_times):.2f}s")
            print(f"  Std:    {np.std(total_times):.2f}s")
            print(f"  Min:    {np.min(total_times):.2f}s")
            print(f"  Max:    {np.max(total_times):.2f}s")
            
            print(f"\nAverage planning time per stage:")
            print(f"  Mean:   {np.mean(planning_times):.3f}s")
            print(f"  Std:    {np.std(planning_times):.3f}s")
            
            print(f"\nAverage execution time per stage:")
            print(f"  Mean:   {np.mean(execution_times):.2f}s")
            print(f"  Std:    {np.std(execution_times):.2f}s")
        
        if failures > 0:
            print("\n" + "-"*70)
            print("FAILURE ANALYSIS")
            print("-"*70)
            
            failure_reasons = Counter([r['failure_reason'] for r in self.failed])
            for reason, count in failure_reasons.most_common():
                percentage = (count / failures) * 100
                print(f"  {reason}: {count} ({percentage:.1f}%)")
        
        print("\n" + "="*70 + "\n")
    
    def generate_plots(self, plot_dir='phase2_plots'):
        """Generate visualization plots"""
        try:
            import matplotlib.pyplot as plt
            import matplotlib
            matplotlib.use('Agg')  # Non-interactive backend
        except ImportError:
            print("✗ Matplotlib not available. Install with: pip3 install matplotlib")
            return
        
        if not self.results:
            print("No data to plot")
            return
        
        # Create output directory
        os.makedirs(plot_dir, exist_ok=True)
        
        # Plot 1: Success Rate
        self._plot_success_rate(plot_dir)
        
        # Plot 2: Time Distribution
        if self.successful:
            self._plot_time_distribution(plot_dir)
        
        # Plot 3: Target Position Visualization
        self._plot_target_positions(plot_dir)
        
        # Plot 4: Failure Reasons
        if self.failed:
            self._plot_failure_reasons(plot_dir)
        
        print(f"✓ Plots saved to: {plot_dir}/")
    
    def _plot_success_rate(self, plot_dir):
        """Plot success/failure pie chart"""
        import matplotlib.pyplot as plt
        
        fig, ax = plt.subplots(figsize=(8, 6))
        
        successes = len(self.successful)
        failures = len(self.failed)
        
        colors = ['#2ecc71', '#e74c3c']
        labels = [f'Success\n{successes} trials', f'Failed\n{failures} trials']
        
        ax.pie([successes, failures], labels=labels, colors=colors,
               autopct='%1.1f%%', startangle=90, textprops={'fontsize': 12})
        ax.set_title('Phase 2: Grasp Success Rate', fontsize=14, fontweight='bold')
        
        plt.tight_layout()
        plt.savefig(f"{plot_dir}/success_rate.png", dpi=150)
        plt.close()
        print(f"  ✓ success_rate.png")
    
    def _plot_time_distribution(self, plot_dir):
        """Plot time distribution histograms"""
        import matplotlib.pyplot as plt
        
        fig, axes = plt.subplots(1, 3, figsize=(15, 4))
        
        total_times = [r['total_time'] for r in self.successful]
        planning_times = [r['avg_planning_time'] for r in self.successful]
        execution_times = [r['avg_execution_time'] for r in self.successful]
        
        # Total time
        axes[0].hist(total_times, bins=20, color='#3498db', edgecolor='black', alpha=0.7)
        axes[0].set_xlabel('Total Time (s)', fontsize=11)
        axes[0].set_ylabel('Frequency', fontsize=11)
        axes[0].set_title(f'Total Grasp Time\nMean: {np.mean(total_times):.2f}s', fontsize=12)
        axes[0].grid(axis='y', alpha=0.3)
        
        # Planning time
        axes[1].hist(planning_times, bins=20, color='#e67e22', edgecolor='black', alpha=0.7)
        axes[1].set_xlabel('Avg Planning Time (s)', fontsize=11)
        axes[1].set_ylabel('Frequency', fontsize=11)
        axes[1].set_title(f'Avg Planning Time per Stage\nMean: {np.mean(planning_times):.3f}s', fontsize=12)
        axes[1].grid(axis='y', alpha=0.3)
        
        # Execution time
        axes[2].hist(execution_times, bins=20, color='#9b59b6', edgecolor='black', alpha=0.7)
        axes[2].set_xlabel('Avg Execution Time (s)', fontsize=11)
        axes[2].set_ylabel('Frequency', fontsize=11)
        axes[2].set_title(f'Avg Execution Time per Stage\nMean: {np.mean(execution_times):.2f}s', fontsize=12)
        axes[2].grid(axis='y', alpha=0.3)
        
        plt.tight_layout()
        plt.savefig(f"{plot_dir}/time_distribution.png", dpi=150)
        plt.close()
        print(f"  ✓ time_distribution.png")
    
    def _plot_target_positions(self, plot_dir):
        """Plot target positions in 3D workspace"""
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d import Axes3D
        
        fig = plt.figure(figsize=(12, 5))
        
        # 3D scatter
        ax1 = fig.add_subplot(121, projection='3d')
        
        for result in self.results:
            color = '#2ecc71' if result['success'] else '#e74c3c'
            marker = 'o' if result['success'] else 'x'
            ax1.scatter(result['target_x'], result['target_y'], result['target_z'],
                       c=color, marker=marker, s=50, alpha=0.6)
        
        ax1.set_xlabel('X (m)', fontsize=10)
        ax1.set_ylabel('Y (m)', fontsize=10)
        ax1.set_zlabel('Z (m)', fontsize=10)
        ax1.set_title('Target Positions in Workspace', fontsize=12, fontweight='bold')
        
        # Legend
        from matplotlib.patches import Patch
        legend_elements = [
            Patch(facecolor='#2ecc71', label='Success'),
            Patch(facecolor='#e74c3c', label='Failed')
        ]
        ax1.legend(handles=legend_elements, loc='upper right')
        
        # 2D top view (X-Y plane)
        ax2 = fig.add_subplot(122)
        
        for result in self.results:
            color = '#2ecc71' if result['success'] else '#e74c3c'
            marker = 'o' if result['success'] else 'x'
            ax2.scatter(result['target_x'], result['target_y'],
                       c=color, marker=marker, s=50, alpha=0.6)
        
        ax2.set_xlabel('X (m)', fontsize=10)
        ax2.set_ylabel('Y (m)', fontsize=10)
        ax2.set_title('Top View (X-Y Plane)', fontsize=12, fontweight='bold')
        ax2.grid(True, alpha=0.3)
        ax2.legend(handles=legend_elements, loc='upper right')
        ax2.set_aspect('equal')
        
        plt.tight_layout()
        plt.savefig(f"{plot_dir}/target_positions.png", dpi=150)
        plt.close()
        print(f"  ✓ target_positions.png")
    
    def _plot_failure_reasons(self, plot_dir):
        """Plot failure reasons bar chart"""
        import matplotlib.pyplot as plt
        
        failure_reasons = Counter([r['failure_reason'] for r in self.failed])
        
        if not failure_reasons:
            return
        
        fig, ax = plt.subplots(figsize=(10, 6))
        
        reasons = list(failure_reasons.keys())
        counts = list(failure_reasons.values())
        
        bars = ax.barh(reasons, counts, color='#e74c3c', edgecolor='black', alpha=0.7)
        
        # Add count labels
        for i, (bar, count) in enumerate(zip(bars, counts)):
            ax.text(count + 0.1, i, str(count), va='center', fontsize=10)
        
        ax.set_xlabel('Number of Failures', fontsize=11)
        ax.set_title('Failure Reasons Distribution', fontsize=12, fontweight='bold')
        ax.grid(axis='x', alpha=0.3)
        
        plt.tight_layout()
        plt.savefig(f"{plot_dir}/failure_reasons.png", dpi=150)
        plt.close()
        print(f"  ✓ failure_reasons.png")


def main():
    parser = argparse.ArgumentParser(
        description='Analyze Phase 2 grasp pipeline results'
    )
    parser.add_argument('csv_file', type=str,
                       help='Path to Phase 2 results CSV file')
    parser.add_argument('--plot', '-p', action='store_true',
                       help='Generate visualization plots')
    parser.add_argument('--plot-dir', type=str, default=None,
                       help='Directory for plot images (default: phase2_plots/ in project dir)')
    
    args = parser.parse_args()
    
    analyzer = Phase2Analyzer(args.csv_file)
    analyzer.compute_statistics()
    
    if args.plot:
        print("Generating plots...")
        # Use test_results directory if plot-dir not specified
        plot_dir = args.plot_dir
        if plot_dir is None:
            # Get project directory from CSV file location
            csv_dir = os.path.dirname(os.path.abspath(args.csv_file))
            plot_dir = os.path.join(csv_dir, 'phase2_plots')
        analyzer.generate_plots(plot_dir)
        print()


if __name__ == '__main__':
    main()
