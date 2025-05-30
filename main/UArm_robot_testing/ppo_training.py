from env import UArmEnv
from stable_baselines3 import PPO
import numpy as np

# Load trained model
model = PPO.load("ppo_uarm")

# Create env (render=True if you want to watch)
env = UArmEnv(render=False)

# Parameters
num_episodes = 500
log_interval = 100  # Print stats every 100 episodes

# Tracking
episode_rewards = []

for ep in range(1, num_episodes + 1):
    obs = env.reset()
    done = False
    total_reward = 0

    while not done:
        action, _ = model.predict(obs)
        obs, reward, done, _ = env.step(action)
        total_reward += reward

    episode_rewards.append(total_reward)

    # Every 100 episodes, print stats
    if ep % log_interval == 0:
        avg_reward = np.mean(episode_rewards[-log_interval:])
        print(f"📦 Episode {ep}/{num_episodes} | Avg Reward (last {log_interval}): {round(avg_reward, 4)}")

env.close()