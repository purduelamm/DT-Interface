import time
import gymnasium
import numpy as np
import manipulator_mujoco

from trajectory_following_num import MPC_pathgen

waypoints = np.zeros((7, 5000))

# Create the environment with rendering in human mode
env = gymnasium.make('manipulator_mujoco/CRX10IALEnv-v0', render_mode='human')

# Reset the environment with a specific seed for reproducibility
observation, info = env.reset(seed=42)

i = -1
# waypoints[0, :] = -109 * np.pi / 180
waypoints[1:5, :1000] = np.array([[16,40,0,30]]).repeat(repeats=1000, axis=0).T * np.pi / 180
waypoints[1:5, 1000:2000] = np.array([[-47,-20,0,30]]).repeat(repeats=1000, axis=0).T * np.pi / 180
waypoints[1:5, 2000:3000] = np.array([[16,40,0,30]]).repeat(repeats=1000, axis=0).T * np.pi / 180
waypoints[1:5, 3000:] = 0
waypoints = waypoints.T

# print(waypoints.shape)

while True:
    if i < 3999:
        i += 1

    # Take a step in the environment using the chosen action
    observation, reward, terminated, truncated, info = env.step(waypoints[i])

    if i == 1:
        time.sleep(5)

    # Check if the episode is over (terminated) or max steps reached (truncated)
    if terminated or truncated:
        # If the episode ends or is truncated, reset the environment
        observation, info = env.reset()

# Close the environment when the simulation is done
env.close()
