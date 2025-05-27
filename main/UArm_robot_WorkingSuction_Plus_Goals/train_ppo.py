from stable_baselines3 import PPO
from env import UArmEnv

env = UArmEnv(render=False)
model = PPO("MlpPolicy", env, verbose=1)
model.learn(total_timesteps=100_000)
model.save("ppo_reach_goal")
