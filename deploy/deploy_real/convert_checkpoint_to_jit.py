
import torch
import torch.nn as nn
import numpy as np

def _maybe_build_obs_normalizer_from_state_dict(state_dict):
    """从 checkpoint 中构建 observation normalizer。"""
    prefix = "actor_obs_normalizer."
    mean_k = prefix + "_mean"
    std_k = prefix + "_std"
    if mean_k not in state_dict or std_k not in state_dict:
        return None

    mean = state_dict[mean_k].detach().clone().float()
    std = state_dict[std_k].detach().clone().float()
    
    # 确保是一维或者二维(1, dim)
    if mean.ndim == 2 and mean.shape[0] == 1:
        mean = mean[0]
    if std.ndim == 2 and std.shape[0] == 1:
        std = std[0]

    eps = 1e-8

    class _ObsNorm(nn.Module):
        def __init__(self, mean_t, std_t):
            super().__init__()
            self.register_buffer("mean", mean_t)
            self.register_buffer("std", std_t)

        def forward(self, x):
            return (x - self.mean) / (self.std + 1e-8)

    print(f"Built ObsNorm with dim={mean.shape[0]}")
    return _ObsNorm(mean, std)


def _build_actor_from_rslrl_state_dict(state_dict):
    """从 checkpoint 中构建 Actor 网络结构。"""
    # Find linear layer indices in the Sequential.
    layer_ids = []
    for k in state_dict.keys():
        if k.startswith("actor.") and k.endswith(".weight"):
            try:
                idx = int(k.split(".")[1])
            except Exception:
                continue
            layer_ids.append(idx)
    layer_ids = sorted(set(layer_ids))
    
    if not layer_ids:
        raise RuntimeError("无法从 checkpoint 推断 actor 层结构。")

    # Infer sizes from weight matrices.
    # For Linear: weight shape = (out_features, in_features)
    first_w = state_dict[f"actor.{layer_ids[0]}.weight"]
    obs_dim = int(first_w.shape[1])
    hidden_outs = [int(state_dict[f"actor.{i}.weight"].shape[0]) for i in layer_ids]

    # All but last are hidden dims; last is action_dim.
    hidden_dims = hidden_outs[:-1]
    action_dim = hidden_outs[-1]

    class _Actor(nn.Module):
        def __init__(self):
            super().__init__()
            layers = []
            in_dim = obs_dim
            for h in hidden_dims:
                layers.append(nn.Linear(in_dim, h))
                layers.append(nn.ELU())
                in_dim = h
            layers.append(nn.Linear(in_dim, action_dim))
            self.actor = nn.Sequential(*layers)

            # Keep std/log_std for compatibility but we don't use it in deterministic policy
            if "std" in state_dict:
                std_init = state_dict["std"].detach().clone()
                self.std = nn.Parameter(std_init)
            elif "log_std" in state_dict:
                log_std = state_dict["log_std"].detach().clone()
                self.std = nn.Parameter(torch.exp(log_std))
            else:
                self.std = nn.Parameter(torch.ones(action_dim))

        def forward(self, obs):
            return self.actor(obs)

    model = _Actor()
    print(f"Built Actor: obs_dim={obs_dim}, hidden={hidden_dims}, action_dim={action_dim}")
    return model

class PolicyWrapper(nn.Module):
    """
    包装 ObsNormalizer 和 Actor，以便导出为一个单一的 JIT 模型。
    """
    def __init__(self, normalizer, actor):
        super().__init__()
        self.normalizer = normalizer
        self.actor = actor
    
    def forward(self, obs):
        if self.normalizer is not None:
            obs = self.normalizer(obs)
        return self.actor(obs)


if __name__ == "__main__":
    path = "/home/lgl/Hexapod_Sim2Real/deploy/pre_train/hexapod/model_2500.pt"
    save_path = "/home/lgl/Hexapod_Sim2Real/deploy/pre_train/hexapod/policy_2500.pt"

    print(f"Loading checkpoint from: {path}")
    checkpoint = torch.load(path, map_location='cpu')
    
    if "model_state_dict" not in checkpoint:
        raise RuntimeError("Checkpoint does not contain 'model_state_dict'")
    
    state_dict = checkpoint["model_state_dict"]

    # 1. Build Normalizer
    normalizer = _maybe_build_obs_normalizer_from_state_dict(state_dict)
    
    # 2. Build Actor
    actor = _build_actor_from_rslrl_state_dict(state_dict)
    
    # 3. Load weights into actor
    # 注意：_Actor 内部定义了 self.actor (Sequential) 和 self.std
    # checkpoint 中的 key 是 "actor.0.weight" 等。
    # strict=False 因为 checkpoint 还包含 critic.* 和 actor_obs_normalizer.*
    missing, unexpected = actor.load_state_dict(state_dict, strict=False)
    print("Actor load_state_dict result:")
    print(f"  Missing: {len(missing)}") # 应该是 critic 和 normalizer 相关的 key
    print(f"  Unexpected: {len(unexpected)}") # 应该是 0
    
    # 4. Wrap them
    policy = PolicyWrapper(normalizer, actor)
    policy.eval() # Set to eval mode!

    # 5. Trace the model to export to TorchScript
    # We need a dummy input to trace.
    # Get input dimension from the first layer weight of actor
    first_layer_weight = state_dict["actor.0.weight"]
    input_dim = first_layer_weight.shape[1] 
    
    dummy_input = torch.zeros(1, input_dim)
    print(f"Tracing with input shape: {dummy_input.shape}")

    traced_policy = torch.jit.trace(policy, dummy_input)
    
    print(f"Saving JIT model to: {save_path}")
    torch.jit.save(traced_policy, save_path)
    print("Done!")
