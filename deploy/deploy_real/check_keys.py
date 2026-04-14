
import torch

path = "/home/lgl/Hexapod_Sim2Real/deploy/pre_train/hexapod_tethered/model_28000.pt"
ckpt = torch.load(path, map_location='cpu')

print("Keys in ckpt:", ckpt.keys())

if 'model_state_dict' in ckpt:
    msd = ckpt['model_state_dict']
    print("Keys in model_state_dict sample:")
    for k in list(msd.keys())[:20]:
        print(k)
        
    has_norm = any("actor_obs_normalizer" in k for k in msd.keys())
    print(f"Has actor_obs_normalizer? {has_norm}")
