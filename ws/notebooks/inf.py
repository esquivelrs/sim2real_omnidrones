import torch
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F
import numpy as np
import os
import sys
from tensordict.tensordict import TensorDict, TensorDictBase
from torchrl.data import (
    BinaryDiscreteTensorSpec,
    CompositeSpec,
    UnboundedContinuousTensorSpec,
    BoundedTensorSpec,
)

model_path = "model/checkpoint_hoverRand_cf.pt"
model_path = "model/full_checkpoint_full_16384.pt"


device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
state_dict = torch.load(model_path, map_location=device)
print(state_dict.keys())

print(state_dict["policy"])