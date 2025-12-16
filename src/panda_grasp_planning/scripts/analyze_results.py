#!/usr/bin/env python3
"""
Phase 3: Benchmark Results Analysis Tool
Analyzes benchmark results and generates detailed reports with visualizations.

This script:
1. Loads benchmark CSV data
2. Computes detailed statistics
3. Generates plots (success rate, planning time distribution, path length, etc.)
4. Creates a comprehensive analysis report

Usage:
    python3 analyze_results.py benchmark_results_20231203_143022.csv
    python3 analyze_results.py benchmark_results.csv --plot --output report.txt
"""

import sys
import argparse
import csv
import numpy as np
from collections import Counter
import os


class BenchmarkAnalyzer:
    def __init__(self, csv_file):
        """
        Initialize analyzer with benchmark CSV file.
        
        Args:
            csv_file: Path to benchmark results CSV
        """
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
                    # Convert types
                    row['trial'] = int(row['trial'])
                    row['target_x'] = float(row['target_x'])
                    row['target_y'] = float(row['target_y'])
                    row['target_z'] = float(row['target_z'])
                    row['success'] = row['success'].lower() == 'true'
                    row['planning_time'] = float(row['planning_time'])
                    row['path_length'] = float(row['path_length'])
                    row['num_waypoints'] = int(row['num_waypoints']) if row['num_waypoints'] else 0
                    
                    self.results.append(row)
                    
                    if row['success']:
                        self.successful.append(row)
                    else:
                        self.failed.append(row)
            
            print(f"✓ Loaded {len(self.results)} trials")
            print(f"  Successful: {len(self.successful)}")
            print(f"  Failed: {len(self.failed)}")
            
        except FileNotFoundError:
            print(f"✗ Error: File not found: {self.csv_file}")
            sys.exit(1)
        except Exception as e:
            print(f"✗ Error loading data: {e}")
            sys.exit(1)
    
    def compute_statistics(self):
        """Compute comprehensive statistics"""
        if not self.results:
            print("No data to analyze")
            return {}
        
        stats = {
            'total_trials': len(self.results),
            'successful_trials': len(self.successful),
            'failed_trials': len(self.failed),
            'success_rate': len(self.successful) / len(self.results) * 100
        }
        
        if self.successful:
            planning_times = [r['planning_time'] for r in self.successful]
            path_lengths = [r['path_length'] for r in self.successful]
            waypoints = [r['num_waypoints'] for r in self.successful]
            
            stats['planning_time'] = {
                'mean': np.mean(planning_times),
                'std': np.std(planning_times),
                'min': np.min(planning_times),
                'max': np.max(planning_times),
                'median': np.median(planning_times),
                'percentile_95': np.percentile(planning_times, 95)
            }
            
            stats['path_length'] = {
                'mean': np.mean(path_lengths),
                'std': np.std(path_lengths),
                'min': np.min(path_lengths),
                'max': np.max(path_lengths),
                'median': np.median(path_lengths)
            }
            
            stats['waypoints'] = {
                'mean': np.mean(waypoints),
                'std': np.std(waypoints),
                'min': int(np.min(waypoints)),
                'max': int(np.max(waypoints)),
                'median': np.median(waypoints)
            }
        
        if self.failed:
            error_counts = Counter([r['error_type'] for r in self.failed])
            stats['failure_modes'] = dict(error_counts)
        
        return stats
    
    def analyze_workspace_coverage(self):
        """Analyze spatial distribution of successful/failed trials"""
        if not self.results:
            return {}
        
        successful_positions = np.array([
            [r['target_x'], r['target_y'], r['target_z']] 
            for r in self.successful
        ])
        
        failed_positions = np.array([
            [r['target_x'], r['target_y'], r['target_z']] 
            for r in self.failed
        ]) if self.failed else np.array([])
        
        coverage = {
            'successful_positions': successful_positions,
            'failed_positions': failed_positions
        }
        
        if len(successful_positions) > 0:
            coverage['successful_centroid'] = np.mean(successful_positions, axis=0)
            coverage['successful_std'] = np.std(successful_positions, axis=0)
        
        if len(failed_positions) > 0:
            coverage['failed_centroid'] = np.mean(failed_positions, axis=0)
            coverage['failed_std'] = np.std(failed_positions, axis=0)
        
        return coverage
    
    def print_report(self, output_file=None):
        """
        Print comprehensive analysis report.
        
        Args:
            output_file: If provided, write report to file instead of stdout
        """
        stats = self.compute_statistics()
        coverage = self.analyze_workspace_coverage()
        
        # Build report text
        report_lines = []
        report_lines.append("=" * 70)
        report_lines.append("RRT BENCHMARK ANALYSIS REPORT")
        report_lines.append("=" * 70)
        report_lines.append(f"Data file: {self.csv_file}")
        report_lines.append(f"Total trials: {stats['total_trials']}")
        report_lines.append("")
        
        # Success/Failure summary
        report_lines.append("-" * 70)
        report_lines.append("SUCCESS RATE")
        report_lines.append("-" * 70)
        report_lines.append(f"Successful: {stats['successful_trials']} "
                          f"({stats['success_rate']:.1f}%)")
        report_lines.append(f"Failed: {stats['failed_trials']} "
                          f"({100 - stats['success_rate']:.1f}%)")
        report_lines.append("")
        
        # Planning time statistics
        if 'planning_time' in stats:
            pt = stats['planning_time']
            report_lines.append("-" * 70)
            report_lines.append("PLANNING TIME STATISTICS (successful trials only)")
            report_lines.append("-" * 70)
            report_lines.append(f"Mean: {pt['mean']:.3f}s")
            report_lines.append(f"Std Dev: {pt['std']:.3f}s")
            report_lines.append(f"Median: {pt['median']:.3f}s")
            report_lines.append(f"Min: {pt['min']:.3f}s")
            report_lines.append(f"Max: {pt['max']:.3f}s")
            report_lines.append(f"95th percentile: {pt['percentile_95']:.3f}s")
            report_lines.append("")
        
        # Path length statistics
        if 'path_length' in stats:
            pl = stats['path_length']
            report_lines.append("-" * 70)
            report_lines.append("PATH LENGTH STATISTICS (joint space, radians)")
            report_lines.append("-" * 70)
            report_lines.append(f"Mean: {pl['mean']:.3f} rad")
            report_lines.append(f"Std Dev: {pl['std']:.3f} rad")
            report_lines.append(f"Median: {pl['median']:.3f} rad")
            report_lines.append(f"Min: {pl['min']:.3f} rad")
            report_lines.append(f"Max: {pl['max']:.3f} rad")
            report_lines.append("")
        
        # Waypoints statistics
        if 'waypoints' in stats:
            wp = stats['waypoints']
            report_lines.append("-" * 70)
            report_lines.append("TRAJECTORY WAYPOINTS")
            report_lines.append("-" * 70)
            report_lines.append(f"Mean: {wp['mean']:.1f}")
            report_lines.append(f"Std Dev: {wp['std']:.1f}")
            report_lines.append(f"Median: {wp['median']:.1f}")
            report_lines.append(f"Min: {wp['min']}")
            report_lines.append(f"Max: {wp['max']}")
            report_lines.append("")
        
        # Failure analysis
        if 'failure_modes' in stats:
            report_lines.append("-" * 70)
            report_lines.append("FAILURE MODE ANALYSIS")
            report_lines.append("-" * 70)
            for error_type, count in sorted(stats['failure_modes'].items(),
                                           key=lambda x: x[1],
                                           reverse=True):
                percentage = count / stats['failed_trials'] * 100
                report_lines.append(f"{error_type}: {count} ({percentage:.1f}%)")
            report_lines.append("")
        
        # Workspace coverage
        if 'successful_centroid' in coverage:
            sc = coverage['successful_centroid']
            ss = coverage['successful_std']
            report_lines.append("-" * 70)
            report_lines.append("WORKSPACE COVERAGE (successful trials)")
            report_lines.append("-" * 70)
            report_lines.append(f"Centroid: X={sc[0]:.3f}, Y={sc[1]:.3f}, Z={sc[2]:.3f}")
            report_lines.append(f"Std Dev:  X={ss[0]:.3f}, Y={ss[1]:.3f}, Z={ss[2]:.3f}")
            report_lines.append("")
        
        if 'failed_centroid' in coverage:
            fc = coverage['failed_centroid']
            fs = coverage['failed_std']
            report_lines.append("-" * 70)
            report_lines.append("WORKSPACE COVERAGE (failed trials)")
            report_lines.append("-" * 70)
            report_lines.append(f"Centroid: X={fc[0]:.3f}, Y={fc[1]:.3f}, Z={fc[2]:.3f}")
            report_lines.append(f"Std Dev:  X={fs[0]:.3f}, Y={fs[1]:.3f}, Z={fs[2]:.3f}")
            report_lines.append("")
        
        # Key insights
        report_lines.append("-" * 70)
        report_lines.append("KEY INSIGHTS")
        report_lines.append("-" * 70)
        
        if stats['success_rate'] > 90:
            report_lines.append(f"✓ Excellent success rate ({stats['success_rate']:.1f}%)")
        elif stats['success_rate'] > 70:
            report_lines.append(f"○ Good success rate ({stats['success_rate']:.1f}%)")
        else:
            report_lines.append(f"✗ Low success rate ({stats['success_rate']:.1f}%) - "
                              "consider workspace constraints or IK solver")
        
        if 'planning_time' in stats:
            if stats['planning_time']['mean'] < 1.0:
                report_lines.append(f"✓ Fast planning (mean: {stats['planning_time']['mean']:.3f}s)")
            elif stats['planning_time']['mean'] < 3.0:
                report_lines.append(f"○ Moderate planning speed (mean: {stats['planning_time']['mean']:.3f}s)")
            else:
                report_lines.append(f"✗ Slow planning (mean: {stats['planning_time']['mean']:.3f}s)")
        
        if 'failure_modes' in stats and stats['failure_modes']:
            top_failure = max(stats['failure_modes'].items(), key=lambda x: x[1])
            report_lines.append(f"⚠ Most common failure: {top_failure[0]} ({top_failure[1]} cases)")
        
        report_lines.append("=" * 70)
        
        # Output report
        report_text = "\n".join(report_lines)
        
        if output_file:
            try:
                with open(output_file, 'w') as f:
                    f.write(report_text)
                print(f"✓ Report saved to: {output_file}")
            except Exception as e:
                print(f"✗ Error saving report: {e}")
                print("\n" + report_text)
        else:
            print("\n" + report_text)
    
    def create_plots(self, output_dir='plots'):
        """
        Create visualization plots (requires matplotlib).
        
        Args:
            output_dir: Directory to save plot images
        """
        try:
            import matplotlib
            matplotlib.use('Agg')  # Non-interactive backend
            import matplotlib.pyplot as plt
        except ImportError:
            print("✗ matplotlib not available. Skipping plots.")
            print("  Install with: pip3 install matplotlib")
            return
        
        # Create output directory
        os.makedirs(output_dir, exist_ok=True)
        
        stats = self.compute_statistics()
        
        # Plot 1: Success rate pie chart
        if stats['total_trials'] > 0:
            fig, ax = plt.subplots(figsize=(8, 6))
            labels = ['Success', 'Failure']
            sizes = [stats['successful_trials'], stats['failed_trials']]
            colors = ['#2ecc71', '#e74c3c']
            explode = (0.1, 0)
            
            ax.pie(sizes, explode=explode, labels=labels, colors=colors,
                  autopct='%1.1f%%', shadow=True, startangle=90)
            ax.set_title('RRTConnect Success Rate', fontsize=16, fontweight='bold')
            
            plt.savefig(os.path.join(output_dir, 'success_rate.png'), dpi=150, bbox_inches='tight')
            print(f"✓ Saved: {output_dir}/success_rate.png")
            plt.close()
        
        # Plot 2: Planning time distribution
        if self.successful:
            planning_times = [r['planning_time'] for r in self.successful]
            
            fig, ax = plt.subplots(figsize=(10, 6))
            ax.hist(planning_times, bins=30, color='#3498db', alpha=0.7, edgecolor='black')
            ax.axvline(np.mean(planning_times), color='red', linestyle='dashed', 
                      linewidth=2, label=f'Mean: {np.mean(planning_times):.3f}s')
            ax.axvline(np.median(planning_times), color='green', linestyle='dashed',
                      linewidth=2, label=f'Median: {np.median(planning_times):.3f}s')
            ax.set_xlabel('Planning Time (seconds)', fontsize=12)
            ax.set_ylabel('Frequency', fontsize=12)
            ax.set_title('Planning Time Distribution (Successful Trials)', 
                        fontsize=14, fontweight='bold')
            ax.legend()
            ax.grid(True, alpha=0.3)
            
            plt.savefig(os.path.join(output_dir, 'planning_time_dist.png'), 
                       dpi=150, bbox_inches='tight')
            print(f"✓ Saved: {output_dir}/planning_time_dist.png")
            plt.close()
        
        # Plot 3: Path length distribution
        if self.successful:
            path_lengths = [r['path_length'] for r in self.successful]
            
            fig, ax = plt.subplots(figsize=(10, 6))
            ax.hist(path_lengths, bins=30, color='#9b59b6', alpha=0.7, edgecolor='black')
            ax.axvline(np.mean(path_lengths), color='red', linestyle='dashed',
                      linewidth=2, label=f'Mean: {np.mean(path_lengths):.3f} rad')
            ax.set_xlabel('Path Length (radians)', fontsize=12)
            ax.set_ylabel('Frequency', fontsize=12)
            ax.set_title('Path Length Distribution', fontsize=14, fontweight='bold')
            ax.legend()
            ax.grid(True, alpha=0.3)
            
            plt.savefig(os.path.join(output_dir, 'path_length_dist.png'),
                       dpi=150, bbox_inches='tight')
            print(f"✓ Saved: {output_dir}/path_length_dist.png")
            plt.close()
        
        # Plot 4: Workspace visualization (2D projections)
        coverage = self.analyze_workspace_coverage()
        
        if len(coverage['successful_positions']) > 0:
            fig, axes = plt.subplots(1, 3, figsize=(15, 5))
            
            suc_pos = coverage['successful_positions']
            fail_pos = coverage.get('failed_positions', np.array([]))
            
            # XY projection
            axes[0].scatter(suc_pos[:, 0], suc_pos[:, 1], 
                          c='green', alpha=0.6, s=20, label='Success')
            if len(fail_pos) > 0:
                axes[0].scatter(fail_pos[:, 0], fail_pos[:, 1],
                              c='red', alpha=0.6, s=20, label='Failure')
            axes[0].set_xlabel('X (m)')
            axes[0].set_ylabel('Y (m)')
            axes[0].set_title('XY Projection')
            axes[0].legend()
            axes[0].grid(True, alpha=0.3)
            
            # XZ projection
            axes[1].scatter(suc_pos[:, 0], suc_pos[:, 2],
                          c='green', alpha=0.6, s=20, label='Success')
            if len(fail_pos) > 0:
                axes[1].scatter(fail_pos[:, 0], fail_pos[:, 2],
                              c='red', alpha=0.6, s=20, label='Failure')
            axes[1].set_xlabel('X (m)')
            axes[1].set_ylabel('Z (m)')
            axes[1].set_title('XZ Projection')
            axes[1].legend()
            axes[1].grid(True, alpha=0.3)
            
            # YZ projection
            axes[2].scatter(suc_pos[:, 1], suc_pos[:, 2],
                          c='green', alpha=0.6, s=20, label='Success')
            if len(fail_pos) > 0:
                axes[2].scatter(fail_pos[:, 1], fail_pos[:, 2],
                              c='red', alpha=0.6, s=20, label='Failure')
            axes[2].set_xlabel('Y (m)')
            axes[2].set_ylabel('Z (m)')
            axes[2].set_title('YZ Projection')
            axes[2].legend()
            axes[2].grid(True, alpha=0.3)
            
            plt.suptitle('Workspace Coverage', fontsize=16, fontweight='bold')
            plt.tight_layout()
            plt.savefig(os.path.join(output_dir, 'workspace_coverage.png'),
                       dpi=150, bbox_inches='tight')
            print(f"✓ Saved: {output_dir}/workspace_coverage.png")
            plt.close()
        
        print(f"\n✓ All plots saved to: {output_dir}/")


def main():
    parser = argparse.ArgumentParser(
        description='Analyze RRT benchmark results and generate report'
    )
    parser.add_argument('csv_file', type=str,
                       help='Path to benchmark results CSV file')
    parser.add_argument('--output', '-o', type=str, default=None,
                       help='Output file for report (default: print to console)')
    parser.add_argument('--plot', '-p', action='store_true',
                       help='Generate visualization plots')
    parser.add_argument('--plot-dir', type=str, default=None,
                       help='Directory for plot images (default: plots/ in project dir)')
    
    args = parser.parse_args()
    
    try:
        print("\n" + "="*70)
        print("RRT BENCHMARK ANALYSIS")
        print("="*70 + "\n")
        
        analyzer = BenchmarkAnalyzer(args.csv_file)
        analyzer.print_report(output_file=args.output)
        
        if args.plot:
            print("\nGenerating plots...")
            # Use test_results directory if plot-dir not specified
            plot_dir = args.plot_dir
            if plot_dir is None:
                # Get project directory from CSV file location
                csv_dir = os.path.dirname(os.path.abspath(args.csv_file))
                plot_dir = os.path.join(csv_dir, 'plots')
            analyzer.create_plots(output_dir=plot_dir)
        
        print("\n✓ Analysis complete!\n")
        
    except Exception as e:
        print(f"\n✗ Error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == '__main__':
    main()
