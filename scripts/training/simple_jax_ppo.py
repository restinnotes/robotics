"""
Simple PPO implementation in JAX/Flax
Standalone, no complex dependencies.
"""

import jax
import jax.numpy as jnp
import flax
import flax.linen as nn
from flax.training.train_state import TrainState
import optax
import numpy as np
import time

class ActorCritic(nn.Module):
    action_dim: int
    activation: str = "tanh"

    @nn.compact
    def __call__(self, x):
        if self.activation == "tanh":
            act = nn.tanh
        else:
            act = nn.relu

        # Actor
        actor_mean = nn.Dense(64)(x)
        actor_mean = act(actor_mean)
        actor_mean = nn.Dense(64)(actor_mean)
        actor_mean = act(actor_mean)
        actor_mean = nn.Dense(self.action_dim)(actor_mean)

        actor_logstd = self.param("log_std", nn.initializers.zeros, (1, self.action_dim))

        # Critic
        critic = nn.Dense(64)(x)
        critic = act(critic)
        critic = nn.Dense(64)(critic)
        critic = act(critic)
        critic = nn.Dense(1)(critic)

        return actor_mean, actor_logstd, critic

class PPOAgent:
    def __init__(self, env, learning_rate=3e-4, gamma=0.99, clip_eps=0.2, entropy_coef=0.01):
        self.env = env
        self.obs_dim = env.observation_space.shape[0]
        self.action_dim = env.action_space.shape[0]
        self.gamma = gamma
        self.clip_eps = clip_eps
        self.entropy_coef = entropy_coef

        # Init Network
        self.network = ActorCritic(action_dim=self.action_dim)
        dummy_obs = jnp.ones((1, self.obs_dim))
        params = self.network.init(jax.random.PRNGKey(0), dummy_obs)

        self.tx = optax.adam(learning_rate)
        self.train_state = TrainState.create(
            apply_fn=self.network.apply,
            params=params,
            tx=self.tx,
        )

        self.predict_fn = jax.jit(self._predict)
        self.train_step_fn = jax.jit(self._train_step)

    def _predict(self, params, obs, rng):
        mean, log_std, value = self.network.apply(params, obs)
        std = jnp.exp(log_std)

        # Sample
        if rng is not None:
            noise = jax.random.normal(rng, shape=mean.shape)
            action = mean + noise * std
        else:
            action = mean

        return action, value

    def get_action(self, obs, rng=None):
        obs = jnp.expand_dims(obs, 0) # Add batch dim
        action, value = self.predict_fn(self.train_state.params, obs, rng)
        return np.array(action[0]), np.array(value[0])

    def _train_step(self, state, batch):
        def loss_fn(params):
            obs, acts, old_log_probs, adv, ret = batch
            mean, log_std, values = self.network.apply(params, obs)
            std = jnp.exp(log_std)
            values = values.squeeze()

            # Log prob
            dist_log_probs = -0.5 * (((acts - mean) / (std + 1e-8))**2 + 2 * log_std + np.log(2 * np.pi))
            log_probs = dist_log_probs.sum(axis=-1)

            # Ratio
            ratio = jnp.exp(log_probs - old_log_probs)

            # PPO Loss
            surr1 = ratio * adv
            surr2 = jnp.clip(ratio, 1.0 - self.clip_eps, 1.0 + self.clip_eps) * adv
            actor_loss = -jnp.minimum(surr1, surr2).mean()

            # Critic Loss
            critic_loss = jnp.mean((ret - values)**2)

            # Entropy
            entropy = (0.5 + 0.5 * np.log(2 * np.pi) + log_std).sum(axis=-1).mean()

            total_loss = actor_loss + 0.5 * critic_loss - self.entropy_coef * entropy
            return total_loss, (actor_loss, critic_loss, entropy)

        grad_fn = jax.value_and_grad(loss_fn, has_aux=True)
        (loss, (a_loss, c_loss, ent)), grads = grad_fn(state.params)
        new_state = state.apply_gradients(grads=grads)
        return new_state, loss, a_loss, c_loss, ent

    def train(self, total_timesteps):
        print(f"JAX PPO Training Start... Steps: {total_timesteps}")

        obs, _ = self.env.reset()
        rng = jax.random.PRNGKey(int(time.time()))

        batch_obs = []
        batch_acts = []
        batch_log_probs = []
        batch_rews = []
        batch_vals = []
        batch_dones = []

        update_steps = 2048

        for step in range(total_timesteps):
            # 1. Action
            rng, key = jax.random.split(rng)
            action, value = self.get_action(obs, key)

            # Log prob calculation (needed for PPO)
            # Re-calculating outside JIT for simplicity or inside?
            # Ideally we get log_prob from _predict, but let's just run step.

            # 2. Step
            next_obs, reward, terminated, truncated, _ = self.env.step(action)
            done = terminated or truncated

            # Store
            batch_obs.append(obs)
            batch_acts.append(action)
            batch_rews.append(reward)
            batch_vals.append(value)
            batch_dones.append(done)
            # Rough log prob (will recalculate in update or store here)
            # Let's simple PPO: calculate later or mock here

            obs = next_obs
            if done:
                obs, _ = self.env.reset()

            # Update
            if (step + 1) % update_steps == 0:
                # Process Batch
                b_obs = jnp.array(batch_obs)
                b_acts = jnp.array(batch_acts)
                b_rews = np.array(batch_rews)
                b_vals = np.array(batch_vals)
                b_dones = np.array(batch_dones)

                # GAE
                advs = np.zeros_like(b_rews)
                gaes = 0
                next_val = 0 # Approximation
                for t in reversed(range(len(b_rews))):
                    if t == len(b_rews) - 1:
                        nextnonterminal = 1.0 - b_dones[t]
                        nextvalues = next_val
                    else:
                        nextnonterminal = 1.0 - b_dones[t]
                        nextvalues = b_vals[t+1]

                    delta = b_rews[t] + self.gamma * nextvalues * nextnonterminal - b_vals[t]
                    gaes = delta + self.gamma * 0.95 * nextnonterminal * gaes
                    advs[t] = gaes

                returns = advs + b_vals

                # Re-calc old log probs
                # We need a helper to get log probs efficiently
                # Skipping for this ultra-simple demo, assumes near-zero shift initially
                # Wait, PPO needs old_log_probs.
                # Let's just create a `get_log_prob` function

                mean, log_std, _ = self.network.apply(self.train_state.params, b_obs)
                std = jnp.exp(log_std)
                dist_log_probs = -0.5 * (((b_acts - mean) / (std + 1e-8))**2 + 2 * log_std + np.log(2 * np.pi))
                b_old_log_probs = dist_log_probs.sum(axis=-1)

                # Train epochs
                batch_data = (b_obs, b_acts, b_old_log_probs, jnp.array(advs), jnp.array(returns))

                # Simple full-batch update (can minibatch if needed)
                for _ in range(10):
                    self.train_state, loss, al, cl, ent = self.train_step_fn(self.train_state, batch_data)

                print(f"Step {step+1}: Loss={loss:.4f} Actor={al:.4f} Critic={cl:.4f}")

                # Clear
                batch_obs = []
                batch_acts = []
                batch_rews = []
                batch_vals = []
                batch_dones = []
                batch_log_probs = []

        print("Training Complete.")

    def save(self, path):
        # Save params using flax serialization (simple bytes)
        with open(path, "wb") as f:
            f.write(flax.serialization.to_bytes(self.train_state.params))

    def load(self, path):
        with open(path, "rb") as f:
            bytes_data = f.read()
        self.train_state = self.train_state.replace(
            params=flax.serialization.from_bytes(self.train_state.params, bytes_data)
        )
