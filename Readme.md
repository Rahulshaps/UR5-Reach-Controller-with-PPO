
# UR5 Reach Controller with PPO (Isaac Lab, PyTorch/SKRL)

## Project Summary
This project trained a UR5 robot arm with a Robotiq 2F-140 gripper to reach randomized 6 DoF target poses inside its workspace using reinforcement learning. The system ran in NVIDIA Isaac Lab with vectorized simulation and learned a closed loop policy via Proximal Policy Optimization (PPO) implemented in PyTorch using the SKRL library.

## Task Definition
- **Objective**: Move the end effector from a random start to a random target pose within the reachable workspace.
- **Success**: Position and orientation errors fall below task thresholds for a minimum hold time.
- **Failure**: Time limit exceeded, joint or velocity limits violated, or simulation instability detected.

## Environment and Dynamics
- **Simulator**: Isaac Lab (PhysX backend), headless vectorized rollout for high throughput.
- **Robot**: UR5 with Robotiq 2F-140 model and standard DH based kinematics in sim.
- **Randomization**: Targets sampled uniformly inside a bounded workspace; reset noise applied to initial joint states.
- **Constraints**: Joint position and velocity limits enforced; contact and collision handling managed by PhysX.

## Observation and Action Spaces
- **Observations**:
  - Joint positions and velocities
  - Previous action (for smoothness)
  - Target pose encoded in robot base frame (position and quaternion)
  - Optional task progress indicators (e.g., time remaining)
- **Actions**:
  - Joint position commands (delta or absolute, bounded per joint)
  - Rate limits to prevent large step changes

## Reward Design
The reward combined several terms to balance accuracy and stability:
- **Position error**: Negative L2 distance between end effector and target position
- **Orientation error**: Quaternion based mismatch penalty
- **Smoothness**: Penalty on action rate to discourage jerky motion
- **Velocity regularization**: Penalty on joint speeds to reduce oscillations
- **Terminal bonuses**: Optional positive bonus on success, penalties on timeout

## Policy and Algorithm
- **Algorithm**: PPO (clipped objective, GAE advantage estimation)
- **Policy network**: Lightweight MLP (2 x 64 hidden units, ELU)
- **Value network**: Same topology as actor for symmetry
- **Training regime**: Large batches from many parallel environments to stabilize updates

## Evaluation
- **Rollouts**: Unseen targets used for validation
- **Metrics**:
  - Success rate under task thresholds
  - Final position and orientation error distributions
  - Action smoothness and step count to success
- **Visualization**: Trajectory plots and time series of errors to inspect convergence and stability

## Results Snapshot
The learned policy consistently reached random target poses within the defined tolerances in simulation. Validation on unseen targets showed generalization across the workspace with smooth, stable motions under the shaped reward.

## Assumptions and Limitations
- Results were obtained in simulation and may require additional calibration, safety limits, and latency handling for real hardware.
- Reward weights and thresholds influence motion style and convergence speed; these were tuned empirically for stable behavior.
- Action space was joint based; task space control or hybrid schemes could be explored for finer end effector regulation.

## Extensions
- Add grasping or visual servoing tasks on top of reaching
- Introduce depth or RGB sensing and train sensor conditioned policies
- Apply domain randomization for sim to real transfer
- Explore alternative algorithms (SAC, TD3) and larger network backbones
