
import torch
import sys

path = "/home/lgl/Hexapod_Sim2Real/deploy/pre_train/hexapod_tethered/model_28000.pt"

print(f"Testing loading: {path}")

try:
    print("Trying torch.load()...")
    content = torch.load(path, map_location='cpu')
    print("Success with torch.load()!")
    print(f"Type: {type(content)}")
    if isinstance(content, dict):
        print(f"Keys: {content.keys()}")
    sys.exit(0)
except Exception as e:
    print(f"Failed with torch.load(): {e}")

try:
    print("Trying torch.jit.load()...")
    model = torch.jit.load(path, map_location='cpu')
    print("Success with torch.jit.load()!")
except Exception as e:
    print(f"Failed with torch.jit.load(): {e}")

