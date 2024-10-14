from typing import Tuple

import numpy as np


def delta_phi(ticks: int, prev_ticks: int, resolution: int) -> Tuple[float, float]:
    """
    Args:
        ticks: Current tick count from the encoders.
        prev_ticks: Previous tick count from the encoders.
        resolution: Number of ticks per full wheel rotation returned by the encoder.
    Return:
        dphi: Rotation of the wheel in radians.
        ticks: current number of ticks.
    """

    # # TODO: these are random values, you have to implement your own solution in here
    # ticks = prev_ticks + int(np.random.uniform(0, 10))
    # dphi = np.random.random()
    # # ---
    delta_ticks = ticks - prev_ticks

    # Calculate the angular resolution (rotation per tick)
    alpha = 2 * np.pi / resolution

    # Calculate the rotation of the wheel (in radians)
    dphi = delta_ticks * alpha
    return dphi, ticks


def estimate_pose(
    R: float,
    baseline: float,
    x_prev: float,
    y_prev: float,
    theta_prev: float,
    delta_phi_left: float,
    delta_phi_right: float,
) -> Tuple[float, float, float]:

    """
    Calculate the current Duckiebot pose using the dead-reckoning model.

    Args:
        R:                  radius of wheel (both wheels are assumed to have the same size) - this is fixed in simulation,
                            and will be imported from your saved calibration for the real robot
        baseline:           distance from wheel to wheel; 2L of the theory
        x_prev:             previous x estimate - assume given
        y_prev:             previous y estimate - assume given
        theta_prev:         previous orientation estimate - assume given
        delta_phi_left:     left wheel rotation (rad)
        delta_phi_right:    right wheel rotation (rad)

    Return:
        x_curr:                  estimated x coordinate
        y_curr:                  estimated y coordinate
        theta_curr:              estimated heading
    """

    # # These are random values, replace with your own
    # x_curr = np.random.random()
    # y_curr = np.random.random()
    # theta_curr = np.random.random()
    # # ---
    # Step 1: Calculate the distance each wheel traveled
    d_left = R * delta_phi_left
    d_right = R * delta_phi_right

    # Step 2: Calculate the forward displacement (center distance) and the change in orientation
    d_A_k = (d_right + d_left) / 2  # Average displacement of both wheels
    delta_theta_k = (d_right - d_left) / baseline  # Change in orientation

    # Step 3: Update the robot's position (x, y) in the world frame
    x_curr = x_prev + d_A_k * np.cos(theta_prev)
    y_curr = y_prev + d_A_k * np.sin(theta_prev)

    # Step 4: Update the robot's heading (orientation)
    theta_curr = theta_prev + delta_theta_k
    return x_curr, y_curr, theta_curr
