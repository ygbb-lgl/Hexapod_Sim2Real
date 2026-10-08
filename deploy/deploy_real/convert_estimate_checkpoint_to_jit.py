#!/usr/bin/env python3
"""Convert a single-expert Estimate RSL-RL checkpoint to TorchScript.

The exported module preserves the recurrent deployment interface used by
``deploy_real_hexapod_tethered_plot_torque_data_estimate.py``::

    action, estimator_hidden_next, actor_hidden_next, context, gate = model(
        state, estimator_hidden, actor_hidden
    )

Edit ``CHECKPOINT_PATH`` and ``POLICY_OUTPUT_PATH`` below, then run::

    python convert_estimate_checkpoint_to_jit.py

The raw checkpoint and the exported policy must use the Estimate ablation
architecture: 68-D deployable state, one expert, 10-D estimated context and a
19-D action.
"""

from __future__ import annotations

import os
import tempfile
from pathlib import Path

import torch

from hexapod_tethered_utils.moe_policy import (
    ACTION_DIM,
    CONTACT_LATENT_DIM,
    CONTEXT_DIM,
    EXPLICIT_CONTEXT_DIM,
    HIDDEN_DIM,
    RNN_NUM_LAYERS,
    STATE_DIM,
    export_inference_core,
    load_inference_core,
)


ESTIMATE_NUM_EXPERTS = 1

# ---------------------------------------------------------------------------
# Paths and conversion settings
# ---------------------------------------------------------------------------
# Directly paste the two absolute paths here, exactly like the MoE converter.
CHECKPOINT_PATH = "/home/lgl/mujoco_hexapod/logs_rslrl/model_6000.pt"
POLICY_OUTPUT_PATH = "/home/lgl/Hexapod_Sim2Real/deploy/pre_train/hexapod_tethered_estimate/policy_estimate.pt"

# The output is replaced only after the temporary TorchScript passes all
# recurrent-output comparisons, so rerunning the script is safe.
OVERWRITE_OUTPUT = True
VALIDATION_STEPS = 8
VALIDATION_ATOL = 1e-6
VALIDATION_RTOL = 1e-5


def _validate_export(
    eager: torch.nn.Module,
    scripted_path: Path,
    *,
    steps: int,
    atol: float,
    rtol: float,
) -> dict[str, float]:
    """Compare all recurrent outputs over a deterministic state sequence."""
    if steps < 1:
        raise ValueError("VALIDATION_STEPS must be at least 1")

    scripted = torch.jit.load(str(scripted_path), map_location="cpu").eval()
    eager = eager.cpu().eval()

    eager_estimator_hidden = torch.zeros(RNN_NUM_LAYERS, 1, HIDDEN_DIM)
    eager_actor_hidden = torch.zeros(RNN_NUM_LAYERS, 1, HIDDEN_DIM)
    jit_estimator_hidden = eager_estimator_hidden.clone()
    jit_actor_hidden = eager_actor_hidden.clone()

    names = (
        "action",
        "estimator_hidden",
        "actor_hidden",
        "context",
        "gate",
    )
    max_abs_error = {name: 0.0 for name in names}
    generator = torch.Generator(device="cpu").manual_seed(20260922)

    with torch.inference_mode():
        for frame in range(steps):
            state = torch.randn(1, STATE_DIM, generator=generator)
            eager_outputs = eager(
                state, eager_estimator_hidden, eager_actor_hidden
            )
            jit_outputs = scripted(
                state, jit_estimator_hidden, jit_actor_hidden
            )

            if len(eager_outputs) != 5 or len(jit_outputs) != 5:
                raise RuntimeError(
                    "Estimate inference core must return action, two hidden "
                    "states, context and gate weights."
                )

            for name, eager_value, jit_value in zip(
                names, eager_outputs, jit_outputs
            ):
                if tuple(eager_value.shape) != tuple(jit_value.shape):
                    raise RuntimeError(
                        f"Frame {frame} {name} shape mismatch: "
                        f"eager={tuple(eager_value.shape)}, "
                        f"jit={tuple(jit_value.shape)}"
                    )
                error = float(
                    torch.max(torch.abs(eager_value - jit_value)).item()
                )
                max_abs_error[name] = max(max_abs_error[name], error)
                try:
                    torch.testing.assert_close(
                        jit_value,
                        eager_value,
                        atol=atol,
                        rtol=rtol,
                    )
                except AssertionError as exc:
                    raise RuntimeError(
                        f"TorchScript validation failed at frame {frame} "
                        f"for {name}; max_abs_error={error:.3e}"
                    ) from exc

            eager_estimator_hidden = eager_outputs[1]
            eager_actor_hidden = eager_outputs[2]
            jit_estimator_hidden = jit_outputs[1]
            jit_actor_hidden = jit_outputs[2]

    expected_shapes = {
        "action": (1, ACTION_DIM),
        "estimator_hidden": (RNN_NUM_LAYERS, 1, HIDDEN_DIM),
        "actor_hidden": (RNN_NUM_LAYERS, 1, HIDDEN_DIM),
        "context": (1, CONTEXT_DIM),
        "gate": (1, ESTIMATE_NUM_EXPERTS),
    }
    for name, output in zip(names, jit_outputs):
        if tuple(output.shape) != expected_shapes[name]:
            raise RuntimeError(
                f"Unexpected exported {name} shape: {tuple(output.shape)}; "
                f"expected {expected_shapes[name]}"
            )
    return max_abs_error


def main() -> None:
    # Accept both Path objects and plain strings so the two paths above can be
    # edited by directly pasting quoted absolute paths.
    checkpoint_path = Path(CHECKPOINT_PATH).expanduser().resolve()
    output_path = Path(POLICY_OUTPUT_PATH).expanduser().resolve()

    if not checkpoint_path.is_file():
        raise FileNotFoundError(f"Checkpoint does not exist: {checkpoint_path}")
    if checkpoint_path == output_path:
        raise ValueError("The output path must differ from the raw checkpoint")
    if output_path.exists() and not OVERWRITE_OUTPUT:
        raise FileExistsError(
            f"Output already exists: {output_path}\n"
            "Set OVERWRITE_OUTPUT = True at the top of this script to replace it."
        )

    output_path.parent.mkdir(parents=True, exist_ok=True)
    print(f"Loading raw one-expert Estimate checkpoint: {checkpoint_path}")
    core = load_inference_core(
        checkpoint_path,
        device="cpu",
        num_experts=ESTIMATE_NUM_EXPERTS,
    )
    print(
        "Validated Estimate architecture: "
        f"state={STATE_DIM}, action={ACTION_DIM}, "
        f"GRU hidden={HIDDEN_DIM}, experts={ESTIMATE_NUM_EXPERTS}, "
        f"context={EXPLICIT_CONTEXT_DIM}+{CONTACT_LATENT_DIM}"
    )

    temporary_path: Path | None = None
    try:
        with tempfile.NamedTemporaryFile(
            prefix=f".{output_path.stem}.",
            suffix=".tmp.pt",
            dir=output_path.parent,
            delete=False,
        ) as temporary_file:
            temporary_path = Path(temporary_file.name)

        export_inference_core(core, temporary_path)
        print(
            "Validating eager and TorchScript outputs for "
            f"{VALIDATION_STEPS} recurrent frames..."
        )
        max_errors = _validate_export(
            core,
            temporary_path,
            steps=VALIDATION_STEPS,
            atol=VALIDATION_ATOL,
            rtol=VALIDATION_RTOL,
        )
        os.replace(temporary_path, output_path)
        temporary_path = None
    finally:
        if temporary_path is not None:
            temporary_path.unlink(missing_ok=True)

    error_summary = ", ".join(
        f"{name}={error:.3e}" for name, error in max_errors.items()
    )
    print(f"Validation passed. Maximum absolute errors: {error_summary}")
    print(f"Saved recurrent one-expert Estimate TorchScript: {output_path}")
    print(f"File size: {output_path.stat().st_size / (1024 * 1024):.2f} MiB")


if __name__ == "__main__":
    main()
