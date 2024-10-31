FROM nvcr.io/nvidia/omniverse/ov-kit-kernel:106.2.0-release.146576.9df8bbcd

COPY ./docker/isaacsim/extensions.toml /home/ubuntu/.nvidia-omniverse/extensions.toml
COPY ./docker/isaacsim/extensions.toml /home/ubuntu/.local/extensions.toml
COPY ./docker/isaacsim/extensions.toml /home/ubuntu/.cache/extensions.toml