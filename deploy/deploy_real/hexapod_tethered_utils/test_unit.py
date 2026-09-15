import numpy as np

def unit_direction_from_yaw_pitch(yaw: float, pitch: float) -> np.ndarray:
    """Return body-frame [x-forward, y-left, z-up] tether direction."""
    horizontal = np.cos(float(pitch))
    direction = np.array(
        [
            horizontal * np.cos(float(yaw)),
            horizontal * np.sin(float(yaw)),
            np.sin(float(pitch)),
        ],
        dtype=np.float32,
    )
    return direction / max(float(np.linalg.norm(direction)), 1e-8)


if __name__ == "__main__":
    # Test the unit direction function with some example yaw and pitch values
    test_yaw = 0 
    test_pitch = 0 
    direction = unit_direction_from_yaw_pitch(test_yaw, test_pitch)
    print("Unit direction vector:", direction)