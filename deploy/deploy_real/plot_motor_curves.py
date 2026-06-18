#!/usr/bin/env python3
"""
Plot motor position tracking curves from logged CSV files.
Usage: python plot_motor_curves.py <log_directory>

Example:
    python plot_motor_curves.py ./motor_logs_20260322_143052

Joint classification (based on hexapod.yaml):
  HAA joints (hip abduction/adduction): policy_idx [0, 3, 6, 9, 12, 15]
  HFE joints (hip flexion/extension):   policy_idx [1, 4, 7, 10, 13, 16]
  KFE joints (knee flexion/extension):  policy_idx [2, 5, 8, 11, 14, 17]
"""

import matplotlib.pyplot as plt
import pandas as pd
import os
import sys
import glob

# Joint classification based on hexapod.yaml
JOINT_GROUPS = {
    'HAA': [0, 3, 6, 9, 12, 15],   # Hip Abduction/Adduction
    'HFE': [1, 4, 7, 10, 13, 16],  # Hip Flexion/Extension
    'KFE': [2, 5, 8, 11, 14, 17],  # Knee Flexion/Extension
}

JOINT_NAMES = {
    0: 'RF_HAA', 1: 'RF_HFE', 2: 'RF_KFE',
    3: 'RM_HAA', 4: 'RM_HFE', 5: 'RM_KFE',
    6: 'RB_HAA', 7: 'RB_HFE', 8: 'RB_KFE',
    9: 'LF_HAA', 10: 'LF_HFE', 11: 'LF_KFE',
    12: 'LM_HAA', 13: 'LM_HFE', 14: 'LM_KFE',
    15: 'LB_HAA', 16: 'LB_HFE', 17: 'LB_KFE',
}

def plot_all_motors(log_dir):
    """Plot tracking curves grouped by joint type: HAA, HFE, KFE."""
    
    # Find all CSV files in the directory
    csv_pattern = os.path.join(log_dir, '*.csv')
    csv_files = sorted(glob.glob(csv_pattern))
    
    if not csv_files:
        print(f"No CSV files found in {log_dir}")
        print(f"Searched pattern: {csv_pattern}")
        return
    
    print(f"Found {len(csv_files)} motor log files")
    
    # Parse all CSV files and organize by policy_idx
    motor_data = {}
    for csv_file in csv_files:
        basename = os.path.basename(csv_file)
        parts = basename.replace('.csv', '').split('_')
        
        try:
            motor_id = parts[1]
            policy_idx = int(parts[4]) if len(parts) > 4 else int(parts[3])
            motor_data[policy_idx] = {
                'csv_file': csv_file,
                'motor_id': motor_id,
                'policy_idx': policy_idx,
                'joint_name': JOINT_NAMES.get(policy_idx, f'Unknown_{policy_idx}')
            }
        except (IndexError, ValueError) as e:
            print(f"Warning: Could not parse {basename}: {e}")
    
    # Group by joint type
    for joint_type, policy_indices in JOINT_GROUPS.items():
        print(f"\nGenerating {joint_type} joint plot...")
        
        fig, axes = plt.subplots(2, 3, figsize=(18, 10))
        axes = axes.flatten()
        
        for local_idx, policy_idx in enumerate(policy_indices):
            if policy_idx not in motor_data:
                print(f"  Warning: No data for policy_idx {policy_idx}")
                continue
            
            data = motor_data[policy_idx]
            csv_file = data['csv_file']
            motor_id = data['motor_id']
            joint_name = data['joint_name']
            
            try:
                df = pd.read_csv(csv_file)
                
                ax = axes[local_idx]
                
                # Plot target position
                ax.plot(df['time_step'], df['target_pos'], 'r-', linewidth=2, label='Target Pos')
                
                # Plot actual position
                ax.plot(df['time_step'], df['actual_pos'], 'b-', linewidth=1.5, label='Actual Pos')
                
                # Plot error (secondary y-axis)
                ax_error = ax.twinx()
                ax_error.plot(df['time_step'], df['error'], 'g--', linewidth=1, alpha=0.7, label='Error')
                
                ax.set_xlabel('Time (s)', fontsize=9)
                ax.set_ylabel('Position (rad)', fontsize=9, color='black')
                ax.set_title(f'{joint_name} (Motor {motor_id}, IDX {policy_idx})\nError: mean={df["error"].mean():.4f}, std={df["error"].std():.4f}', 
                            fontsize=10)
                ax.grid(True, alpha=0.3)
                ax.tick_params(labelsize=8)
                
                # Combine legends
                lines1, labels1 = ax.get_legend_handles_labels()
                lines2, labels2 = ax_error.get_legend_handles_labels()
                ax.legend(lines1 + lines2, labels1 + labels2, loc='upper right', fontsize=8)
                
                ax_error.set_ylabel('Error (rad)', fontsize=8, color='green')
                ax_error.tick_params(labelsize=8, colors='green')
                
            except Exception as e:
                print(f"Error reading {csv_file}: {e}")
                axes[local_idx].text(0.5, 0.5, f'Error\n{os.path.basename(csv_file)}', 
                              ha='center', va='center', fontsize=10)
                axes[local_idx].grid(True)
        
        plt.suptitle(f'{joint_type} Joints - Tracking Performance', fontsize=14, fontweight='bold', y=1.02)
        plt.tight_layout()
        
        # Save figure
        output_file = os.path.join(log_dir, f'{joint_type}_joints_tracking.png')
        try:
            plt.savefig(output_file, dpi=150, bbox_inches='tight')
            print(f"Saved {joint_type} plot to {output_file}")
        except PermissionError:
            # Fallback: save to current directory
            fallback_file = os.path.join(os.getcwd(), f'{joint_type}_joints_tracking.png')
            print(f"Permission denied: {output_file}")
            print(f"Saving to current directory instead: {fallback_file}")
            plt.savefig(fallback_file, dpi=150, bbox_inches='tight')
        
        plt.show()
    
    print(f"\nGenerated 3 plots (HAA, HFE, KFE)")


def plot_individual_motor(csv_file):
    """Plot detailed curve for a single motor."""
    
    if not os.path.exists(csv_file):
        print(f"File not found: {csv_file}")
        return
    
    df = pd.read_csv(csv_file)
    
    fig, axes = plt.subplots(3, 1, figsize=(14, 10))
    
    # Plot 1: Target vs Actual position
    axes[0].plot(df['time_step'], df['target_pos'], 'r-', linewidth=2, label='Target')
    axes[0].plot(df['time_step'], df['actual_pos'], 'b-', linewidth=1.5, label='Actual')
    axes[0].set_ylabel('Position (rad)')
    axes[0].set_title(f'Motor Tracking Performance\nMean Error: {df["error"].mean():.6f} rad, Std: {df["error"].std():.6f} rad')
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)
    
    # Plot 2: Tracking error
    axes[1].plot(df['time_step'], df['error'], 'g-', linewidth=1.5)
    axes[1].axhline(y=0, color='r', linestyle='--', linewidth=0.5)
    axes[1].set_ylabel('Error (rad)')
    axes[1].grid(True, alpha=0.3)
    
    # Plot 3: Policy action
    axes[2].plot(df['time_step'], df['action'], 'purple', linewidth=1.5)
    axes[2].set_xlabel('Time (s)')
    axes[2].set_ylabel('Action (normalized)')
    axes[2].grid(True, alpha=0.3)
    
    plt.tight_layout()
    
    # Save individual plot
    output_file = csv_file.replace('.csv', '_curve.png')
    plt.savefig(output_file, dpi=150, bbox_inches='tight')
    print(f"Saved individual plot to {output_file}")
    
    plt.show()


def plot_comparison_motors(log_dir, motor_indices=None):
    """Compare tracking performance across multiple motors."""
    
    if motor_indices is None:
        motor_indices = list(range(18))
    
    csv_pattern = os.path.join(log_dir, 'motor_*_policy_idx_*.csv')
    csv_files = sorted(glob.glob(csv_pattern))
    
    fig, axes = plt.subplots(2, 1, figsize=(16, 10))
    
    colors = plt.cm.tab10.colors
    
    for idx, csv_file in enumerate(csv_files):
        basename = os.path.basename(csv_file)
        parts = basename.replace('.csv', '').split('_')
        motor_id = parts[1]
        policy_idx = int(parts[4])
        
        if policy_idx not in motor_indices:
            continue
        
        df = pd.read_csv(csv_file)
        
        # Plot error comparison
        color = colors[policy_idx % len(colors)]
        axes[0].plot(df['time_step'], df['error'], linewidth=1.5, 
                    label=f'Motor {motor_id} (IDX {policy_idx})', color=color, alpha=0.8)
        
        # Plot action comparison
        axes[1].plot(df['time_step'], df['action'], linewidth=1.5, 
                    label=f'Motor {motor_id} (IDX {policy_idx})', color=color, alpha=0.8)
    
    axes[0].set_ylabel('Tracking Error (rad)')
    axes[0].set_title('Comparison: All Motors Tracking Error')
    axes[0].legend(loc='upper right', fontsize=8)
    axes[0].grid(True, alpha=0.3)
    axes[0].axhline(y=0, color='black', linestyle='-', linewidth=0.5)
    
    axes[1].set_xlabel('Time (s)')
    axes[1].set_ylabel('Policy Action (normalized)')
    axes[1].set_title('Comparison: All Motors Policy Actions')
    axes[1].legend(loc='upper right', fontsize=8)
    axes[1].grid(True, alpha=0.3)
    
    plt.tight_layout()
    
    output_file = os.path.join(log_dir, 'motors_comparison.png')
    plt.savefig(output_file, dpi=150, bbox_inches='tight')
    print(f"Saved comparison plot to {output_file}")
    
    plt.show()


def print_statistics(log_dir):
    """Print statistics for all motors."""
    
    csv_pattern = os.path.join(log_dir, 'motor_*_policy_idx_*.csv')
    csv_files = sorted(glob.glob(csv_pattern))
    
    print("\n" + "="*80)
    print("MOTOR TRACKING STATISTICS SUMMARY")
    print("="*80)
    print(f"{'Motor ID':<12} {'Policy IDX':<12} {'Mean Error':<15} {'Std Error':<15} {'Max Error':<15}")
    print("-"*80)
    
    stats = []
    for csv_file in csv_files:
        basename = os.path.basename(csv_file)
        parts = basename.replace('.csv', '').split('_')
        motor_id = parts[1]
        policy_idx = parts[4]
        
        df = pd.read_csv(csv_file)
        mean_err = abs(df['error'].mean())
        std_err = df['error'].std()
        max_err = abs(df['error']).max()
        
        stats.append({
            'motor_id': motor_id,
            'policy_idx': policy_idx,
            'mean_error': mean_err,
            'std_error': std_err,
            'max_error': max_err
        })
        
        print(f"Motor {motor_id:<8} {policy_idx:<12} {mean_err:<15.6f} {std_err:<15.6f} {max_err:<15.6f}")
    
    print("="*80)
    
    # Find worst performers
    stats_sorted_by_mean = sorted(stats, key=lambda x: x['mean_error'], reverse=True)
    stats_sorted_by_max = sorted(stats, key=lambda x: x['max_error'], reverse=True)
    
    print("\nTOP 3 MOTORS WITH HIGHEST MEAN ERROR:")
    for i, stat in enumerate(stats_sorted_by_mean[:3], 1):
        print(f"  {i}. Motor {stat['motor_id']} (Policy IDX {stat['policy_idx']}): {stat['mean_error']:.6f} rad")
    
    print("\nTOP 3 MOTORS WITH HIGHEST MAX ERROR:")
    for i, stat in enumerate(stats_sorted_by_max[:3], 1):
        print(f"  {i}. Motor {stat['motor_id']} (Policy IDX {stat['policy_idx']}): {stat['max_error']:.6f} rad")
    
    print("="*80 + "\n")


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print(__doc__)
        print("Available modes:")
        print("  python plot_motor_curves.py <log_dir>              - Plot all motors")
        print("  python plot_motor_curves.py stats <log_dir>        - Print statistics only")
        print("  python plot_motor_curves.py compare <log_dir>      - Comparison plot")
        print("  python plot_motor_curves.py single <csv_file>      - Single motor detail")
        sys.exit(1)
    
    if sys.argv[1] == 'stats' and len(sys.argv) >= 3:
        print_statistics(sys.argv[2])
    elif sys.argv[1] == 'compare' and len(sys.argv) >= 3:
        plot_comparison_motors(sys.argv[2])
    elif sys.argv[1] == 'single' and len(sys.argv) >= 3:
        plot_individual_motor(sys.argv[2])
    else:
        plot_all_motors(sys.argv[1])
