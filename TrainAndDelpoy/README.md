<div align="center">


  <img src="media/header_compress.png" width="800" alt="NVIDIA Isaac GR00T N1.5 Header">
  
  <!-- --- -->
  
  <p style="font-size: 1.2em;">
    <a href="https://developer.nvidia.com/isaac/gr00t"><strong>Website</strong></a> | 
    <a href="https://huggingface.co/nvidia/GR00T-N1.5-3B"><strong>Model</strong></a> |
    <a href="https://huggingface.co/datasets/nvidia/PhysicalAI-Robotics-GR00T-X-Embodiment-Sim"><strong>Dataset</strong></a> |
    <a href="https://arxiv.org/abs/2503.14734"><strong>Paper</strong></a>
  </p>
</div>

[![CI](https://github.com/NVIDIA/Isaac-GR00T/actions/workflows/main.yml/badge.svg)](https://github.com/NVIDIA/Isaac-GR00T/actions/workflows/main.yml)
[![Code style: black](https://img.shields.io/badge/code%20style-black-000000.svg)](https://github.com/psf/black)
[![Imports: isort](https://img.shields.io/badge/%20imports-isort-%231674b1?style=flat&labelColor=ef8336)](https://pycqa.github.io/isort/)
[![GitHub star chart](https://img.shields.io/github/stars/NVIDIA/Isaac-GR00T?style=flat-square)](https://star-history.com/#NVIDIA/Isaac-GR00T)
[![Open Issues](https://img.shields.io/github/issues-raw/NVIDIA/Isaac-GR00T?style=flat-square)](https://github.com/NVIDIA/Isaac-GR00T/issues)

## NVIDIA Isaac GR00T


## Prerequisites

- We have tested the code on Ubuntu 20.04 and 22.04, GPU: H100, L40, RTX 4090, and A6000 for finetuning and Python==3.10, CUDA version 12.4.
- Additionally, successful finetuning and inference have been verified on Ubuntu 22.04 with GPU: RTX 3090 Ti, Python==3.10, CUDA version 11.8.
- For inference, we have tested on Ubuntu 20.04 and 22.04, GPU: RTX 3090, RTX 3090 Ti, RTX 4090, and A6000.
- If you haven't installed CUDA (version 12.4 recommended, but 11.8 also confirmed working), please follow the instructions [here](https://docs.nvidia.com/cuda/cuda-installation-guide-linux/) to install it.
- If you haven't installed tensorrt, please follow the instructions [here](https://docs.nvidia.com/deeplearning/tensorrt/latest/installing-tensorrt/installing.html#) to install it.
- Please make sure you have the following dependencies installed in your system: `ffmpeg`, `libsm6`, `libxext6`

## Installation Guide

Clone the repo:

```sh
git clone https://github.com/song-jisu/Isaac-GR00T-BiSO101.git
cd Isaac-GR00T-BiSO101
```

Create a new conda environment and install the dependencies. We recommend Python 3.10:

> Note: CUDA 12.4 is recommended and officially tested. However, CUDA 11.8 has also been verified to work.
> In such cases, make sure to install a compatible version of `flash-attn` manually (e.g., `flash-attn==2.8.2` was confirmed working with CUDA 11.8).

```sh
conda create -n gr00t python=3.10
conda activate gr00t
pip install --upgrade setuptools
pip install -e .[base]
pip install --no-build-isolation flash-attn==2.7.1.post4 
```

## Getting started with this repo

We provide accessible Jupyter notebooks and detailed documentation in the [`./getting_started`](./getting_started) folder. Utility scripts can be found in the [`./scripts`](./scripts) folder. Additionally, a comprehensive tutorial for finetuning the model on the SO-101 robot is available on [HuggingFace](https://huggingface.co/blog/nvidia/gr00t-n1-5-so101-tuning). I added new data-config to train dual arm SO101 with 3 cameras or 4 cameras. You can simply train with [`./shell`](./shell) folder.

## 0. Fine-Tuning

Users can run the finetuning script below to finetune the model with the example dataset. A tutorial is available in [`getting_started/2_finetuning.ipynb`](getting_started/2_finetuning.ipynb).

 You can finetune your datasets with this command. Before running it, you should change the values to fit your situation.

Then run the finetuning script:
```bash
. shell/finetune.sh
```

**Note**: If you are finetuning on a 4090, you need to pass the `--no-tune_diffusion_model` flag when running `gr00t_finetune.py` to avoid CUDA out of memory.

The recommended finetuning configuration is to boost your batch size to the max, and train for 20k steps.

*Hardware Performance Considerations*
- **Finetuning Performance**: We used 1 H100 node or L40 node for optimal finetuning. Other hardware configurations (e.g. A6000, RTX 4090) will also work but may take longer to converge. The exact batch size is dependent on the hardware, and on which component of the model is being tuned.
- **LoRA finetuning**: We used 2 A6000 GPUs or 2 RTX 4090 GPUs for LoRA finetuning. Users can try out different configurations for effective finetuning.
- **Inference Performance**: For real-time inference, most modern GPUs perform similarly when processing a single sample. Our benchmarks show minimal difference between L40 and RTX 4090 for inference speed.

For new embodiment finetuning, checkout our notebook in [`getting_started/3_0_new_embodiment_finetuning.md`](getting_started/3_0_new_embodiment_finetuning.md).

### Choosing the Right Embodiment Head

<div align="center">
<img src="media/robots-banner.png" width="1000" alt="robots-banner">
</div>

GR00T N1.5 provides three pretrained embodiment heads optimized for different robot configurations:

- **`EmbodimentTag.GR1`**: Designed for humanoid robots with dexterous hands using absolute joint space control
- **`EmbodimentTag.OXE_DROID`**: Optimized for single arm robots using delta end-effector (EEF) control  
- **`EmbodimentTag.AGIBOT_GENIE1`**: Built for humanoid robots with grippers using absolute joint space control
- **`EmbodimentTag.NEW_EMBODIMENT`**: (Non-pretrained) New embodiment head for finetuning on new robot embodiments

Select the embodiment head that best matches your robot's configuration for optimal finetuning performance. For detailed information on the observation and action spaces, see [`EmbodimentTag`](getting_started/4_deeper_understanding.md#embodiment-action-head-fine-tuning).


### Sim Env: [robocasa-gr1-tabletop-tasks](https://github.com/robocasa/robocasa-gr1-tabletop-tasks)

Sample dataset for finetuning can be downloaed from our huggingface [here](https://huggingface.co/datasets/nvidia/PhysicalAI-Robotics-GR00T-X-Embodiment-Sim)

For Simulation Evaluation, please refer to [robocasa-gr1-tabletop-tasks](https://github.com/robocasa/robocasa-gr1-tabletop-tasks)

## 1. Eval with Real Robots

Download the model checkpoint and run the inference service. Before running this command, you should change the values to fit your situation.
```bash
. shell/server.sh
```

On a different terminal, run the client mode to send requests to the server. This will send a random observation to the server and get an action back.
```bash
python scripts/inference_service.py  --client
```

On new terminal, run this command to eval with real robot.
```bash
. shell/eval.sh
```

# Contributing

For more details, see [CONTRIBUTING.md](CONTRIBUTING.md)


## License 

```
# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
```


## Citation

Nvidia Research. [website](https://research.nvidia.com/labs/lpr/publication/gr00tn1_2025/)
```bibtex
@inproceedings{gr00tn1_2025,
  archivePrefix = {arxiv},
  eprint     = {2503.14734},
  title      = {{GR00T} {N1}: An Open Foundation Model for Generalist Humanoid Robots},
  author     = {NVIDIA and Johan Bjorck andFernando Castañeda, Nikita Cherniadev and Xingye Da and Runyu Ding and Linxi "Jim" Fan and Yu Fang and Dieter Fox and Fengyuan Hu and Spencer Huang and Joel Jang and Zhenyu Jiang and Jan Kautz and Kaushil Kundalia and Lawrence Lao and Zhiqi Li and Zongyu Lin and Kevin Lin and Guilin Liu and Edith Llontop and Loic Magne and Ajay Mandlekar and Avnish Narayan and Soroush Nasiriany and Scott Reed and You Liang Tan and Guanzhi Wang and Zu Wang and Jing Wang and Qi Wang and Jiannan Xiang and Yuqi Xie and Yinzhen Xu and Zhenjia Xu and Seonghyeon Ye and Zhiding Yu and Ao Zhang and Hao Zhang and Yizhou Zhao and Ruijie Zheng and Yuke Zhu},
  month      = {March},
  year       = {2025},
  booktitle  = {ArXiv Preprint},
}
```
