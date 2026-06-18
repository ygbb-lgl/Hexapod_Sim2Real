# Motor Debug Logging Guide

## Overview

This guide explains how to use the motor logging and plotting tools to debug whether tracking issues come from the policy or PD control.

## Files Modified

- `deploy_real_hexapod.py`: Added CSV logging for all 18 motors
- `plot_motor_curves.py`: New script to visualize logged data

## How It Works

### Data Logging (Automatic)

When you run `deploy_real_hexapod.py`, it automatically:

1. **Creates a timestamped log directory**: `motor_logs_YYYYMMDD_HHMMSS/`
2. **Opens 18 CSV files**: One for each motor, named `motor_XX_policy_idx_YY.csv`
3. **Records at 50Hz control rate**:
   - `time_step`: Time in seconds
   - `counter`: Step counter
   - `target_pos`: Target position from policy (rad)
   - `actual_pos`: Actual motor position from encoder (rad)
   - `error`: Tracking error = actual - target (rad)
   - `action`: Raw policy action output (normalized [-1, 1])

### CSV File Naming Convention

```
motor_XX_policy_idx_YY.csv
       ↑               ↑
   motor_id      policy index
   (hardware)    (0-17 in buffer)
```

Example: `motor_05_policy_idx_02.csv` logs motor ID 5 at policy index 2.

## Usage

### Step 1: Run Your Robot Control

```bash
cd /home/lgl/Hexapod_Sim2Real/deploy/deploy_real
python deploy_real_hexapod.py
```

Perform your test maneuvers. When finished (press LB to exit), all log files will be saved.

### Step 2: Plot All Motors

```bash
python plot_motor_curves.py ./motor_logs_20260322_143052
```

This creates:
- `all_motors_tracking.png`: 6×3 grid showing all 18 motors
- Individual plots for each motor

### Step 3: View Statistics Summary

```bash
python plot_motor_curves.py stats ./motor_logs_20260322_143052
```

Shows:
- Mean, std, and max error for each motor
- Top 3 worst-performing motors (by mean and max error)

### Step 4: Compare Motors Side-by-Side

```bash
python plot_motor_curves.py compare ./motor_logs_20260322_143052
```

Overlays all motors' errors and actions for comparison.

### Step 5: Inspect Single Motor Detail

```bash
python plot_motor_curves.py single ./motor_logs_20260322_143052/motor_05_policy_idx_02.csv
```

Shows detailed 3-panel plot:
1. Target vs Actual position
2. Tracking error over time
3. Policy action output

## Interpreting Results

### Policy Issues (Look for these patterns):

- **Erratic target positions**: If `target_pos` jumps around unpredictably
- **High-frequency oscillations**: Policy output vibrating at high frequency
- **Unreasonable magnitudes**: Actions consistently at ±1 (clipping)
- **Correlated errors across motors**: Multiple motors showing similar bad patterns

### PD Control Issues (Look for these patterns):

- **Consistent lag**: `actual_pos` follows `target_pos` but with delay
- **Overshoot/oscillation**: Actual position overshoots target, then settles
- **Steady-state error**: Constant offset between target and actual
- **Slow response**: Actual position rises slowly to reach target

### Good Tracking Indicators:

- Mean error < 0.01 rad (~0.57 degrees)
- Max error < 0.05 rad (~2.86 degrees)
- Error distribution centered around 0 (no bias)
- Quick settling time (< 0.2s)

## Tuning Recommendations

### If PD Gains Are Too Low:
- Increase `kp` in `_build_policy_joint_gains()` (currently 120)
- Increase `kd` for damping (currently 1.2)
- Look for sluggish response, large steady-state error

### If PD Gains Are Too High:
- Decrease `kp` and/or `kd`
- Look for oscillations, overshoot, high-frequency vibration

### If Policy Is the Issue:
- Check policy training convergence
- Verify observation normalization matches training
- Consider policy fine-tuning with real-world data

## Example Workflow

```bash
# 1. Run the robot
python deploy_real_hexapod.py

# 2. After test, find the latest log directory
ls -lt motor_logs_* | head -1

# 3. Plot everything
python plot_motor_curves.py ./motor_logs_20260322_143052

# 4. Check statistics
python plot_motor_curves.py stats ./motor_logs_20260322_143052

# 5. Focus on problematic motors
python plot_motor_curves.py single ./motor_logs_20260322_143052/motor_05_policy_idx_02.csv
```

## Tips

1. **Log directory**: Each run creates a new timestamped folder - no data loss
2. **File cleanup**: Old logs accumulate - delete manually when done
3. **Performance**: CSV writing adds negligible overhead (< 0.1ms per cycle)
4. **Memory**: Files grow linearly - for 1 minute at 50Hz: ~3000 rows × 18 files ≈ 5MB total
5. **Safety**: Logs flush automatically - safe to Ctrl+C anytime

## Troubleshooting

### No CSV files created
- Check console output for "[Logging] Created ..." messages
- Verify write permissions in current directory

### Plot script fails
- Ensure matplotlib and pandas are installed:
  ```bash
  pip install matplotlib pandas
  ```

### Missing data points
- Normal if robot was stopped/restarted mid-run
- Gaps indicate control loop interruptions

### Errors look huge (> 1 rad)
- Check motor direction configuration
- Verify `joint2motor_idx` mapping is correct
- Possible encoder wrap-around issue (2π discontinuity)

---

**Quick Start Command:**
```bash
python deploy_real_hexapod.py && python plot_motor_curves.py ./motor_logs_$(ls -td motor_logs_* | head -1)
```
