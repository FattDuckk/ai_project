from stable_baselines3 import PPO
from stable_baselines3.common.monitor import Monitor
from env import UArmEnv

# Create and wrap environment
env = UArmEnv(render=False)
env = Monitor(env, filename="logs/ppo_env_log")  # Logs to CSV for later analysis

# Create PPO model
model = PPO("MlpPolicy", env, verbose=1)

# Train the model
model.learn(total_timesteps=200_000)

# Save the model
model.save("ppo_move_to_goal")
