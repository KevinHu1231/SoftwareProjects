import numpy as np
import gymnasium as gym
from stable_baselines3 import TD3
from stable_baselines3.common.noise import NormalActionNoise, OrnsteinUhlenbeckActionNoise

import torch.nn as nn
import torch
from stable_baselines3.common.torch_layers import BaseFeaturesExtractor
from stable_baselines3.common.preprocessing import get_flattened_obs_dim

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

def interpret_action(self, action):
    if action == 0:
        forward_speed = 0
    elif action == 1:
        climb_speed = 0
    elif action == 2:
        steering_speed = 0

def compute_reward(quad_state, goal_state, quad_vel, action, obstacles, collision_info):
    goal_radius = 3
    omega = [1,1,1]
    safety_dist = 5
    min_dist = 1
    if d(quad_state[:,0],goal_state) < goal_radius:
        reward = 10
    else:
        R_goal = d(quad_state[:,1]) - d(quad_state[:,0])
        C_obs = (safety_dist - d(quad_state[:,0],nearest_obstacle(quad_state[:,0],obstacles)))/(safety_dist - min_dist)
        C_act = action - quad_vel
        C_pos = d(quad_state[:,0],goal_state)
        P_state = omega[0]*C_obs - omega[1]*C_act - omega[2]*C_pos
        reward = R_goal - P_state

def d(quad_state, goal_state):
    return np.linalg.norm(quad_state - goal_state)

def nearest_obstacle(state,obstacles):
    min_dist = np.inf
    min_obstacle = None
    for obstacle in obstacles:
        dist = d(state,obstacle)
        if dist<min_dist:
            min_dist = dist
            min_obstacle = obstacle
    return min_obstacle

env = gym.make("Pendulum-v1", render_mode="rgb_array")

# The noise objects for TD3
n_actions = env.action_space.shape[-1]
action_noise = NormalActionNoise(mean=np.zeros(n_actions), sigma=0.1 * np.ones(n_actions))

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

model.learn(total_timesteps=10000, log_interval=10)
model.save("td3_pendulum")
vec_env = model.get_env()

del model # remove to demonstrate saving and loading

model = TD3.load("td3_pendulum")

obs = vec_env.reset()
while True:
    action, _states = model.predict(obs)
    obs, rewards, dones, info = vec_env.step(action)
    vec_env.render("human")


    # thresh_dist = 7
    # beta = 1

    # z = -10
    # pts = [np.array([-0.55265, -31.9786, -19.0225]),np.array([48.59735, -63.3286, -60.07256]),np.array([193.5974, -55.0786, -46.32256]),np.array([369.2474, 35.32137, -62.5725]),np.array([541.3474, 143.6714, -32.07256]),]

    # quad_pt = np.array(list((self.state["position"].x_val, self.state["position"].y_val,self.state["position"].z_val,)))

    # if self.state["collision"]:
    #     reward = -100
    # else:
    #     dist = 10000000
    #     for i in range(0, len(pts) - 1):
    #         dist = min(dist, np.linalg.norm(np.cross((quad_pt - pts[i]), (quad_pt - pts[i + 1]))) / np.linalg.norm(pts[i] - pts[i + 1]))

    #     if dist > thresh_dist:
    #         reward = -10
    #     else:
    #         reward_dist = math.exp(-beta * dist) - 0.5
    #         reward_speed = (np.linalg.norm([self.state["velocity"].x_val, self.state["velocity"].y_val, self.state["velocity"].z_val,])- 0.5)
    #         reward = reward_dist + reward_speed

# model = DQN(
#     "CnnPolicy",
#     env,
#     learning_rate=0.00025,
#     verbose=1,
#     batch_size=32,
#     train_freq=4,
#     target_update_interval=10000,
#     learning_starts=10000,
#     buffer_size=500000,
#     max_grad_norm=10,
#     exploration_fraction=0.1,
#     exploration_final_eps=0.01,
#     device="cuda",
#     tensorboard_log="./tb_logs/",
# )