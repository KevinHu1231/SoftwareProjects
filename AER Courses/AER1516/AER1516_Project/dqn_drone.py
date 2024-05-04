import setup_path
import gym
import airgym
import time

from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import DummyVecEnv, VecTransposeImage
from stable_baselines3.common.evaluation import evaluate_policy
from stable_baselines3.common.callbacks import EvalCallback

import numpy as np
import gymnasium as gym
from stable_baselines3 import TD3
from stable_baselines3.common.noise import NormalActionNoise, OrnsteinUhlenbeckActionNoise

import torch.nn as nn
import torch
from stable_baselines3.common.torch_layers import BaseFeaturesExtractor
from stable_baselines3.common.preprocessing import get_flattened_obs_dim

# Create a DummyVecEnv for main airsim gym env
env = DummyVecEnv(
    [
        lambda: Monitor(
            gym.make(
                "airgym:airsim-drone-sample-v0",
                ip_address="127.0.0.1",
                step_length=0.25,
                image_shape=(84, 84, 1),
            )
        )
    ]
)

# Wrap env as VecTransposeImage to allow SB to handle frame observations
env = VecTransposeImage(env)

class CustomActorCritic(nn.Module):
    def __init__(self, depth_image, state_pos, actions):
        super(CustomActorCritic, self).__init__()
        # Define custom actor network
        self.actor_1 = nn.Sequential(
            nn.Conv2d(in_channels=1, out_channels=8, kernel_size=3, stride=1, padding=1),
            nn.ReLU(),
            nn.Conv2d(in_channels=8, out_channels=8, kernel_size=3, stride=1, padding=1),
            nn.ReLU(),
            nn.Conv2d(in_channels=8, out_channels=8, kernel_size=3, stride=1, padding=1),
            nn.ReLU(),
            nn.AdaptiveAvgPool3d(1)
        )

        self.actor_2 = nn.Sequential(
            nn.Linear(14, 64),
            nn.ReLU(),
            nn.Linear(64, 32),
            nn.ReLU(),
            nn.Linear(32,3)
        )

        # Define custom critic network
        self.critic = nn.Sequential(
            nn.Linear(depth_image.shape[0] + actions.shape[0], 400),
            nn.ReLU(),
            nn.Linear(400, 300),
            nn.ReLU(),
            nn.Linear(300, 1)
        )  

        self.state_pos = state_pos

    def forward(self, obs, action):
        # Compute actor output
        actor_1_out = self.actor_1(obs)
        actor_2_in = torch.cat([actor_1_out, self.state_pos], dim=1)
        actor_out = self.actor_2(actor_2_in)
        # Concatenate observation and action for critic input
        critic_in = torch.cat([obs, action], dim=1)
        # Compute critic output
        critic_1_out = self.critic(critic_in)
        critic_2_out = self.critic(critic_in)
        return actor_out, critic_1_out, critic_2_out

class CustomPolicy(BaseFeaturesExtractor):
    def __init__(self, observation_space: gym.spaces.Box):
        super(CustomPolicy, self).__init__(observation_space, features_dim=64)
        self.policy_net = CustomActorCritic(observation_space, action_space)

    def forward(self, obs: th.Tensor) -> th.Tensor:
        return self.policy_net.actor(obs)

# The noise objects for TD3
n_actions = env.action_space.shape[-1]
action_noise = NormalActionNoise(mean=np.zeros(n_actions), sigma=0.1 * np.ones(n_actions))


# Initialize RL algorithm type and parameters
model = TD3(
    CustomPolicy,
    env,
    learning_rate=0.00025,
    verbose=1,
    batch_size=32,
    train_freq=4,
    action_noise=action_noise,
    target_update_interval=10000,
    learning_starts=10000,
    buffer_size=500000,
    max_grad_norm=10,
    exploration_fraction=0.1,
    exploration_final_eps=0.01,
    device="cuda",
    tensorboard_log="./tb_logs/",
)

# Create an evaluation callback with the same env, called every 10000 iterations
callbacks = []
eval_callback = EvalCallback(
    env,
    callback_on_new_best=None,
    n_eval_episodes=5,
    best_model_save_path=".",
    log_path=".",
    eval_freq=10000,
)
callbacks.append(eval_callback)

kwargs = {}
kwargs["callback"] = callbacks

# Train for a certain number of timesteps
model.learn(
    total_timesteps=5e5,
    tb_log_name="dqn_airsim_drone_run_" + str(time.time()),
    **kwargs
)

# Save policy weights
model.save("dqn_airsim_drone_policy")
