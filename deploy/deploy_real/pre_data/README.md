# Synthetic 50 Hz pre-tension data

The `synthetic_*.csv` files in this directory are deterministic fixtures for
checking the CSV loader, causal window construction, model training, and online
checkpoint format. They are not measurements from the robot and must not be
used to claim real prediction accuracy.

## Files

- `synthetic_traj_01_50hz.csv`: forward acceleration
- `synthetic_traj_02_50hz.csv`: lateral motion and turning
- `synthetic_traj_03_50hz.csv`: stop-and-go motion
- `synthetic_traj_04_50hz.csv`: backward motion
- `synthetic_traj_05_50hz.csv`: slalom-like motion
- `synthetic_traj_06_50hz.csv`: varying velocity and tension reference
- `synthetic_merged_50hz.csv`: all six trajectories, used for training

Each trajectory contains 600 rows (12 seconds) at an exact 20 ms interval. The
causal indexing matches the real logger: each row contains the previously
executed command `u_(k-1)`, the observed force `F_k`, and the newly issued
command `u_k`.

## Validate and train

From the repository root:

```bash
conda run -n isaacgym python deploy/deploy_real/pre_tension.py inspect-csv \
  --csv deploy/deploy_real/pre_data/synthetic_merged_50hz.csv \
  --expected-dt-s 0.02

conda run -n isaacgym python deploy/deploy_real/pre_tension.py train \
  --csv deploy/deploy_real/pre_data/synthetic_merged_50hz.csv \
  --output /tmp/synthetic_tension_model_50hz.pt \
  --device cpu \
  --expected-dt-s 0.02
```

Regenerate the fixtures with:

```bash
python deploy/deploy_real/generate_synthetic_pre_tension_data.py
python deploy/deploy_real/merge_pre_tension_csv.py \
  --input-dir deploy/deploy_real/pre_data \
  --pattern 'synthetic_traj_*_50hz.csv' \
  --output deploy/deploy_real/pre_data/synthetic_merged_50hz.csv
```

Keep real logger output under its `hexapod_*.csv` names. Merge and train real
data separately from these fixtures.
