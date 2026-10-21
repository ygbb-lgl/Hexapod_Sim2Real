import torch
import torch.nn as nn


def _maybe_build_obs_normalizer_from_state_dict(state_dict):
    """从 MJLab checkpoint 中构建 observation normalizer。"""
    prefix = "obs_normalizer."
    mean_k = prefix + "_mean"
    std_k = prefix + "_std"
    if mean_k not in state_dict or std_k not in state_dict:
        raise RuntimeError("Checkpoint 缺少 HexapodEstimate 的 observation normalizer。")

    mean = state_dict[mean_k].detach().clone().float().reshape(-1)
    std = state_dict[std_k].detach().clone().float().reshape(-1)
    if mean.shape != (63,) or std.shape != (63,):
        raise RuntimeError("HexapodEstimate 的 observation normalizer 必须是 63 维。")

    class _ObsNorm(nn.Module):
        def __init__(self, mean_t, std_t):
            super().__init__()
            self.register_buffer("mean", mean_t)
            self.register_buffer("std", std_t)

        def forward(self, x):
            return (x - self.mean) / (self.std + 0.01)

    print(f"Built ObsNorm with dim={mean.shape[0]}")
    return _ObsNorm(mean, std)


def _build_actor_from_rslrl_state_dict(state_dict):
    """从 checkpoint 中构建 GRU 估计器、估计输出层和 Actor 网络。"""
    if "rnn.rnn.weight_ih_l0" not in state_dict:
        raise RuntimeError("Checkpoint 不包含 GRU，请使用 HexapodEstimate 训练模型。")

    layer_ids = []
    for k in state_dict.keys():
        if k.startswith("mlp.") and k.endswith(".weight"):
            try:
                idx = int(k.split(".")[1])
            except ValueError:
                continue
            layer_ids.append(idx)
    layer_ids = sorted(set(layer_ids))
    if not layer_ids:
        raise RuntimeError("无法从 checkpoint 推断 actor 层结构。")

    first_w = state_dict[f"mlp.{layer_ids[0]}.weight"]
    policy_input_dim = int(first_w.shape[1])
    hidden_outs = [int(state_dict[f"mlp.{i}.weight"].shape[0]) for i in layer_ids]
    hidden_dims = hidden_outs[:-1]
    action_dim = hidden_outs[-1]
    estimator_input_dim = int(state_dict["rnn.rnn.weight_ih_l0"].shape[1])
    rnn_hidden_dim = int(state_dict["rnn.rnn.weight_hh_l0"].shape[1])
    estimator_output_dim = int(state_dict["estimator_head.weight"].shape[0])

    if (estimator_input_dim, estimator_output_dim, policy_input_dim, action_dim) != (
        60, 9, 72, 18
    ):
        raise RuntimeError(
            "HexapodEstimate 应为 63 维观测、60 维 GRU 输入、"
            "9 维估计值、72 维 Actor 输入和 18 维动作。"
        )

    class _Actor(nn.Module):
        def __init__(self):
            super().__init__()
            self.estimator_input_dim = estimator_input_dim
            self.rnn = nn.GRU(estimator_input_dim, rnn_hidden_dim, num_layers=1)
            self.estimator_head = nn.Linear(rnn_hidden_dim, estimator_output_dim)

            layers = []
            in_dim = policy_input_dim
            for h in hidden_dims:
                layers.append(nn.Linear(in_dim, h))
                layers.append(nn.ELU())
                in_dim = h
            layers.append(nn.Linear(in_dim, action_dim))
            self.mlp = nn.Sequential(*layers)
            self.register_buffer("hidden_state", torch.zeros(1, 1, rnn_hidden_dim))

        def forward(self, obs):
            estimator_obs = obs[:, :self.estimator_input_dim].unsqueeze(0)
            latent, hidden_state = self.rnn(estimator_obs, self.hidden_state)
            self.hidden_state[:] = hidden_state
            estimate = self.estimator_head(latent.squeeze(0))
            policy_input = torch.cat((obs, estimate), dim=-1)
            return self.mlp(policy_input)

        @torch.jit.export
        def reset(self):
            self.hidden_state.zero_()

    model = _Actor()
    print(
        f"Built Actor: obs_dim=63, GRU={estimator_input_dim}->{rnn_hidden_dim}, "
        f"estimate_dim={estimator_output_dim}, hidden={hidden_dims}, "
        f"action_dim={action_dim}"
    )
    return model


class PolicyWrapper(nn.Module):
    """包装 ObsNormalizer、GRU 估计器和 Actor，导出为单一的 JIT 模型。"""

    def __init__(self, normalizer, actor):
        super().__init__()
        self.normalizer = normalizer
        self.actor = actor

    def forward(self, obs):
        obs = self.normalizer(obs)
        return self.actor(obs)

    @torch.jit.export
    def reset(self):
        self.actor.reset()


if __name__ == "__main__":
    path = "/home/lgl/Hexapod_Sim2Real/deploy/pre_train/hexapod_estimate/1020_flat/model_24500.pt"
    save_path = "/home/lgl/Hexapod_Sim2Real/deploy/pre_train/hexapod_estimate/1020_flat/policy_24500.pt"

    print(f"Loading checkpoint from: {path}")
    checkpoint = torch.load(path, map_location="cpu", weights_only=True)
    if "actor_state_dict" not in checkpoint:
        raise RuntimeError("Checkpoint does not contain 'actor_state_dict'")
    state_dict = checkpoint["actor_state_dict"]

    # 1. Build Normalizer
    normalizer = _maybe_build_obs_normalizer_from_state_dict(state_dict)

    # 2. Build Actor (GRU + estimator head + MLP)
    actor = _build_actor_from_rslrl_state_dict(state_dict)

    # 3. Load weights into actor
    # MJLab 的 rnn.rnn.* 对应本脚本中的 rnn.*。
    actor_state_dict = {}
    for k, v in state_dict.items():
        if k.startswith("rnn.rnn."):
            actor_state_dict[k.replace("rnn.rnn.", "rnn.", 1)] = v
        elif k.startswith(("estimator_head.", "mlp.")):
            actor_state_dict[k] = v

    missing, unexpected = actor.load_state_dict(actor_state_dict, strict=False)
    print("Actor load_state_dict result:")
    print(f"  Missing: {len(missing)}")  # 只有新建的 hidden_state，不属于训练权重。
    print(f"  Unexpected: {len(unexpected)}")
    if missing != ["hidden_state"] or unexpected:
        raise RuntimeError(f"Actor 权重不匹配：missing={missing}, unexpected={unexpected}")

    # 4. Wrap them
    policy = PolicyWrapper(normalizer, actor)
    policy.eval()

    # 5. Script the model to preserve recurrent state and the reset() method.
    input_dim = normalizer.mean.numel()
    dummy_input = torch.zeros(1, input_dim)
    print(f"Scripting with input shape: {dummy_input.shape}")
    scripted_policy = torch.jit.script(policy)
    with torch.no_grad():
        scripted_policy(dummy_input)
        scripted_policy.reset()  # 保存前清空预运行产生的 GRU 历史。

    print(f"Saving JIT model to: {save_path}")
    torch.jit.save(scripted_policy, save_path)
    print("Done!")
