# YOLOv5 🚀 by Ultralytics, GPL-3.0 license

# Start FROM Nvidia PyTorch image https://ngc.nvidia.com/catalog/containers/nvidia:pytorch
FROM nvidia/cuda:11.7.1-cudnn8-devel-ubuntu20.04

# Install linux packages
RUN apt update && apt install -y zip htop screen libgl1-mesa-glx
RUN apt purge -y python3.8 && rm -rf /opt/conda
RUN apt install -y python3.8
RUN apt install -y python-is-python3
RUN apt update && apt install -y zip htop screen libgl1-mesa-glx python3-pip git

# # Install python dependencies
RUN python -m pip install --upgrade pip
RUN pip uninstall -y nvidia-tensorboard nvidia-tensorboard-plugin-dlprof
RUN pip install coremltools onnx gsutil notebook wandb>=0.12.2

COPY requirements.txt .
ENV SKLEARN_ALLOW_DEPRECATED_SKLEARN_PACKAGE_INSTALL True
RUN pip install --no-cache -r requirements.txt

ENV DEBIAN_FRONTEND noninteractive
RUN apt-get install -y libsm6 libxrender1 libfontconfig1 libglib2.0-0
RUN pip install pyg_lib torch_scatter torch_sparse torch_cluster torch_spline_conv -f https://data.pyg.org/whl/torch-2.0.0+cu117.html
RUN pip install torch==2.0.0 torchvision==0.15.1 torchaudio==2.0.1