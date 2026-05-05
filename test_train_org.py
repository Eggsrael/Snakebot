import math
from dataclasses import dataclass
import gymnasium as gym
import numpy as np
from gymnasium import spaces
from stable_baselines3 import PPO
from stable_baselines3.common.env_checker import check_env

def wrap_angle(angle: float) -> float:
    """Wrap angle to [-pi, pi]."""
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


@dataclass
class RobotState:
    x: float
    y: float
    theta: float
    v: float = 0.0

class RectObstacle:
    def __init__(self, x1, y1, x2, y2):
        self.x1 = min(x1, x2)
        self.y1 = min(y1, y2)
        self.x2 = max(x1, x2)
        self.y2 = max(y1, y2)

    def contains(self, x, y):
        return self.x1 <= x <= self.x2 and self.y1 <= y <= self.y2

class TwoRobotWorld:
    """
    Simple 2D world for leader + follower.
    """

    def __init__(self, world_size=5.0, max_range=1.5):
        self.world_size = world_size
        self.max_range = max_range
        self.robot_radius = 0.12

        self.obstacles = [
            RectObstacle(1.5, 1.5, 1.9, 3.0),
            RectObstacle(3.0, 0.8, 3.4, 2.0),
            RectObstacle(2.2, 3.2, 4.0, 3.6),
        ]

    def collides(self, x, y):
        # Boundary collision
        if x < self.robot_radius or x > self.world_size - self.robot_radius:
            return True
        if y < self.robot_radius or y > self.world_size - self.robot_radius:
            return True

        # Obstacle collision
        for obs in self.obstacles:
            closest_x = clamp(x, obs.x1, obs.x2)
            closest_y = clamp(y, obs.y1, obs.y2)
            dx = x - closest_x
            dy = y - closest_y
            if dx * dx + dy * dy <= self.robot_radius * self.robot_radius:
                return True
        return False

    def ray_cast(self, x, y, theta, max_range=None, step=0.02):
        """
        Very simple ray cast against walls + rectangular obstacles.
        """
        if max_range is None:
            max_range = self.max_range

        dist = 0.0
        while dist <= max_range:
            px = x + dist * math.cos(theta)
            py = y + dist * math.sin(theta)

            # World boundary
            if px < 0 or px > self.world_size or py < 0 or py > self.world_size:
                return dist

            # Obstacles
            for obs in self.obstacles:
                if obs.contains(px, py):
                    return dist

            dist += step

        return max_range

    def tof_triplet(self, robot: RobotState):
        """
        Simulate Left / Center / Right ToF.
        """
        offsets = [math.radians(25), 0.0, math.radians(-25)]
        readings = []
        for a in offsets:
            d = self.ray_cast(robot.x, robot.y, robot.theta + a, self.max_range)
            readings.append(d)
        return np.array(readings, dtype=np.float32)

class LeaderEnv(gym.Env):
    """
    Leader learns obstacle avoidance + smooth forward progress.
    Discrete actions:
      0 = forward
      1 = forward-left
      2 = forward-right
      3 = pivot-left
      4 = pivot-right
      5 = stop
    """

    metadata = {"render_modes": []}

    def __init__(self, episode_steps=300):
        super().__init__()
        self.world = TwoRobotWorld()
        self.episode_steps = episode_steps
        self.step_count = 0

        # obs = [tof_left, tof_center, tof_right, heading_to_goal, dist_to_goal]
        low = np.array([0.0, 0.0, 0.0, -math.pi, 0.0], dtype=np.float32)
        high = np.array(
            [self.world.max_range, self.world.max_range, self.world.max_range, math.pi, 10.0],
            dtype=np.float32,
        )

        self.observation_space = spaces.Box(low=low, high=high, dtype=np.float32)
        self.action_space = spaces.Discrete(6)

        self.robot = None
        self.goal = None

    def _get_obs(self):
        tof = self.world.tof_triplet(self.robot)

        dx = self.goal[0] - self.robot.x
        dy = self.goal[1] - self.robot.y
        goal_theta = math.atan2(dy, dx)
        heading_error = wrap_angle(goal_theta - self.robot.theta)
        dist_to_goal = math.sqrt(dx * dx + dy * dy)

        return np.array(
            [tof[0], tof[1], tof[2], heading_error, dist_to_goal],
            dtype=np.float32,
        )

    def _apply_action(self, action):
        # simple differential-drive style motion
        if action == 0:      # forward
            v = 0.08
            w = 0.0
        elif action == 1:    # forward-left
            v = 0.06
            w = 0.5
        elif action == 2:    # forward-right
            v = 0.06
            w = -0.5
        elif action == 3:    # pivot-left
            v = 0.0
            w = 1.0
        elif action == 4:    # pivot-right
            v = 0.0
            w = -1.0
        else:                # stop
            v = 0.0
            w = 0.0

        dt = 0.15
        new_theta = wrap_angle(self.robot.theta + w * dt)
        new_x = self.robot.x + v * math.cos(new_theta) * dt
        new_y = self.robot.y + v * math.sin(new_theta) * dt

        collision = self.world.collides(new_x, new_y)
        if not collision:
            self.robot.x = new_x
            self.robot.y = new_y
            self.robot.theta = new_theta

        return collision

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        self.step_count = 0

        self.robot = RobotState(x=0.6, y=0.6, theta=0.0)
        self.goal = (4.4, 4.4)

        return self._get_obs(), {}

    def step(self, action):
        self.step_count += 1

        prev_obs = self._get_obs()

        prev_dist = float(prev_obs[4])
        dist_to_goal = float(obs[4])
        heading_error = float(abs(obs[3]))

        collision = self._apply_action(action)
        obs = self._get_obs()
        tof = obs[:3]

        terminated = False
        truncated = False

        # Reward shaping
        reward = 0.0
        reward += (prev_dist - dist_to_goal) * 8.0
        reward += -0.2 * heading_error
        reward += 0.15

        # Obstacle caution
        min_tof = float(np.min(tof))
        if min_tof < 0.2:
            reward -= 0.8

        if collision:
            reward -= 20.0
            terminated = True

        if dist_to_goal < 0.25:
            reward += 30.0
            terminated = True

        if self.step_count >= self.episode_steps:
            truncated = True

        return obs, float(reward), bool(terminated), bool(truncated), {}

    def render(self):
        pass

class FollowerEnv(gym.Env):
    """
    Follower tracks a scripted leader.
    Discrete actions:
      0 = forward
      1 = forward-left
      2 = forward-right
      3 = pivot-left
      4 = pivot-right
      5 = stop

    Observation:
      [tof_left, tof_center, tof_right,
       rel_dist, rel_bearing, yaw_rate_hint, prev_action_norm]
    """

    metadata = {"render_modes": []}

    def __init__(self, episode_steps=300):
        super().__init__()
        self.world = TwoRobotWorld()
        self.episode_steps = episode_steps
        self.step_count = 0
        self.target_follow_dist = 0.5
        self.prev_action = 5

        low = np.array(
            [0.0, 0.0, 0.0, 0.0, -math.pi, -2.0, 0.0],
            dtype=np.float32,
        )
        high = np.array(
            [self.world.max_range, self.world.max_range, self.world.max_range, 10.0, math.pi, 2.0, 1.0],
            dtype=np.float32,
        )

        self.observation_space = spaces.Box(low=low, high=high, dtype=np.float32)
        self.action_space = spaces.Discrete(6)

        self.leader = None
        self.follower = None
        self.leader_phase = 0.0

    def _move_leader_scripted(self):
        """
        Scripted leader motion for follower training.
        """
        dt = 0.15

        # Smooth-ish path
        v = 0.07
        w = 0.5 * math.sin(self.leader_phase)

        self.leader_phase += 0.08
        new_theta = wrap_angle(self.leader.theta + w * dt)
        new_x = self.leader.x + v * math.cos(new_theta) * dt
        new_y = self.leader.y + v * math.sin(new_theta) * dt

        if self.world.collides(new_x, new_y):
            # simple bounce-turn
            self.leader.theta = wrap_angle(self.leader.theta + math.radians(90))
        else:
            self.leader.x = new_x
            self.leader.y = new_y
            self.leader.theta = new_theta

    def _apply_follower_action(self, action):
        if action == 0:
            v = 0.08
            w = 0.0
        elif action == 1:
            v = 0.06
            w = 0.5
        elif action == 2:
            v = 0.06
            w = -0.5
        elif action == 3:
            v = 0.0
            w = 1.0
        elif action == 4:
            v = 0.0
            w = -1.0
        else:
            v = 0.0
            w = 0.0

        dt = 0.15
        new_theta = wrap_angle(self.follower.theta + w * dt)
        new_x = self.follower.x + v * math.cos(new_theta) * dt
        new_y = self.follower.y + v * math.sin(new_theta) * dt

        collision = self.world.collides(new_x, new_y)
        if not collision:
            self.follower.x = new_x
            self.follower.y = new_y
            self.follower.theta = new_theta

        yaw_rate_hint = w
        return collision, yaw_rate_hint

    def _leader_relative_features(self):
        dx = self.leader.x - self.follower.x
        dy = self.leader.y - self.follower.y
        rel_dist = math.sqrt(dx * dx + dy * dy)

        leader_angle = math.atan2(dy, dx)
        rel_bearing = wrap_angle(leader_angle - self.follower.theta)
        return rel_dist, rel_bearing

    def _get_obs(self, yaw_rate_hint=0.0):
        tof = self.world.tof_triplet(self.follower)
        rel_dist, rel_bearing = self._leader_relative_features()
        prev_action_norm = self.prev_action / (self.action_space.n - 1)

        return np.array(
            [
                tof[0],
                tof[1],
                tof[2],
                rel_dist,
                rel_bearing,
                yaw_rate_hint,
                prev_action_norm,
            ],
            dtype=np.float32,
        )

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        self.step_count = 0
        self.prev_action = 5

        self.leader = RobotState(x=1.0, y=1.0, theta=0.0)
        self.follower = RobotState(x=0.4, y=1.0, theta=0.0)
        self.leader_phase = 0.0

        return self._get_obs(0.0), {}

    def step(self, action):
        self.step_count += 1

        prev_rel_dist, prev_rel_bearing = self._leader_relative_features()

        self._move_leader_scripted()
        collision, yaw_rate_hint = self._apply_follower_action(action)
        obs = self._get_obs(yaw_rate_hint)

        rel_dist = float(obs[3])
        rel_bearing = float(obs[4])
        tof = obs[:3]

        terminated = False
        truncated = False

        # Reward: keep target distance, keep leader centered, avoid obstacles
        reward = 0.0
        reward += -2.5 * abs(rel_dist - self.target_follow_dist)
        reward += -1.2 * abs(rel_bearing)

        # small bonus for improvement
        reward += 1.0 * (abs(prev_rel_dist - self.target_follow_dist) - abs(rel_dist - self.target_follow_dist))
        reward += 0.8 * (abs(prev_rel_bearing) - abs(rel_bearing))

        # alive bonus
        reward += 0.2

        # smoothness penalty
        if action != self.prev_action:
            reward -= 0.05

        # near-obstacle penalty
        min_tof = float(np.min(tof))
        if min_tof < 0.2:
            reward -= 1.0

        # lost leader condition
        if rel_dist > 2.5 or abs(rel_bearing) > math.radians(100):
            reward -= 8.0
            terminated = True

        if collision:
            reward -= 20.0
            terminated = True

        if abs(rel_dist - self.target_follow_dist) < 0.08 and abs(rel_bearing) < math.radians(8):
            reward += 0.8

        if self.step_count >= self.episode_steps:
            truncated = True

        self.prev_action = action
        return obs, float(reward), bool(terminated), bool(truncated), {}

    def render(self):
        pass

def train_leader():
    env = LeaderEnv()
    check_env(env, warn=True)

    model = PPO(
        "MlpPolicy",
        env,
        verbose=1,
        learning_rate=3e-4,
        n_steps=1024,
        batch_size=64,
        gamma=0.99,
        gae_lambda=0.95,
        clip_range=0.2,
        ent_coef=0.01,
    )
    model.learn(total_timesteps=100_000)
    model.save("ppo_leader")
    print("Saved leader model to ppo_leader.zip")


def train_follower():
    env = FollowerEnv()
    check_env(env, warn=True)

    model = PPO(
        "MlpPolicy",
        env,
        verbose=1,
        learning_rate=3e-4,
        n_steps=1024,
        batch_size=64,
        gamma=0.99,
        gae_lambda=0.95,
        clip_range=0.2,
        ent_coef=0.01,
    )
    model.learn(total_timesteps=150_000)
    model.save("ppo_follower")
    print("Saved follower model to ppo_follower.zip")


def test_model(model_path, env):
    model = PPO.load(model_path)
    obs, _ = env.reset()

    total_reward = 0.0
    for step in range(300):
        action, _ = model.predict(obs, deterministic=True)
        obs, reward, terminated, truncated, _ = env.step(action)
        total_reward += reward
        print(f"step={step:03d}, action={action}, reward={reward:.3f}, total={total_reward:.3f}")

        if terminated or truncated:
            print("Episode ended.")
            break


if __name__ == "__main__":
    print("1 = Train leader")
    print("2 = Train follower")
    print("3 = Test leader")
    print("4 = Test follower")
    choice = input("Select: ").strip()

    if choice == "1":
        train_leader()
    elif choice == "2":
        train_follower()
    elif choice == "3":
        test_model("ppo_leader", LeaderEnv())
    elif choice == "4":
        test_model("ppo_follower", FollowerEnv())
    else:
        print("Invalid choice.")
