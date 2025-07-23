import gymnasium as gym
import jsbgym
import numpy as np

ENV_ID = ""
env = gym.make("C172X-HeadingControlTask-Shaping.STANDARD-NoFG-v0", render_mode="graph")
env.reset()
env.render()

steps = 0
while steps < 1000:
    action = np.array([0, 0, 0])
    observation, reward, terminated, truncated, info = env.step(action)