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

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import pandas as pd
import os
import sys
import glob
import re

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

def plot_all_motors(log_dir, output_dir=None):
    """Save three 2x3 figures of target and actual joint positions."""
    csv_files = sorted(glob.glob(os.path.join(log_dir, '*_motor_*_policy_idx_*.csv')))
    motor_data = {}
    for csv_file in csv_files:
        match = re.search(r'_motor_(\d+)_policy_idx_(\d+)\.csv$', os.path.basename(csv_file))
        if match:
            motor_id, policy_idx = map(int, match.groups())
            if policy_idx in motor_data:
                raise ValueError(f'Duplicate policy index {policy_idx}: {csv_file}')
            motor_data[policy_idx] = (motor_id, csv_file)

    missing = sorted(set(range(18)) - set(motor_data))
    if missing:
        raise ValueError(f'Missing motor CSV files for policy indices: {missing}')

    output_dir = output_dir or log_dir
    os.makedirs(output_dir, exist_ok=True)
    for joint_type, policy_indices in JOINT_GROUPS.items():
        fig, axes = plt.subplots(2, 3, figsize=(18, 10), sharex=True)
        for ax, policy_idx in zip(axes.flat, policy_indices):
            motor_id, csv_file = motor_data[policy_idx]
            df = pd.read_csv(csv_file)
            required = {'time_step', 'target_pos', 'actual_pos'}
            if not required.issubset(df.columns):
                raise ValueError(f'{csv_file} is missing columns: {sorted(required - set(df.columns))}')
            ax.plot(df['time_step'], df['target_pos'], color='tab:red', label='Target position')
            ax.plot(df['time_step'], df['actual_pos'], color='tab:blue', label='Actual position')
            ax.set_title(f'{JOINT_NAMES[policy_idx]} | Motor {motor_id:02d} | Policy {policy_idx:02d}')
            ax.set_xlabel('Time (s)')
            ax.set_ylabel('Position (rad)')
            ax.grid(alpha=0.3)
            ax.legend(fontsize=8)
        fig.suptitle(f'{joint_type} joint position tracking', fontsize=16)
        fig.tight_layout()
        output_file = os.path.join(output_dir, f'{joint_type}_joints_tracking.png')
        fig.savefig(output_file, dpi=180)
        plt.close(fig)
        print(f'Saved {output_file}')


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
        plot_all_motors(sys.argv[1], sys.argv[2] if len(sys.argv) >= 3 else None)
