# syntax=docker/dockerfile:1
FROM ros:jazzy-ros-base AS build
SHELL ["/bin/bash", "-c"]

# Only the XBot ROS interfaces are needed; no XBotCore or MPC stack is built.
ARG XBOT_MSGS_REF=61f26da74039b921b3acdc61f1c12560dcf4b9d1
RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential cmake git lsb-release && \
    rm -rf /var/lib/apt/lists/* && \
    git init /tmp/xbot_msgs && \
    git -C /tmp/xbot_msgs remote add origin https://github.com/ADVRHumanoids/xbot_msgs.git && \
    git -C /tmp/xbot_msgs fetch --depth 1 origin "${XBOT_MSGS_REF}" && \
    git -C /tmp/xbot_msgs checkout --detach FETCH_HEAD
RUN source /opt/ros/jazzy/setup.bash && \
    cmake -S /tmp/xbot_msgs -B /tmp/xbot-msgs-build \
      -DCMAKE_INSTALL_PREFIX=/opt/kyon-replay -DBUILD_TESTING=OFF && \
    cmake --build /tmp/xbot-msgs-build --parallel 4 && \
    cmake --install /tmp/xbot-msgs-build

COPY . /tmp/kyon_actions
RUN source /opt/ros/jazzy/setup.bash && \
    source /opt/kyon-replay/share/xbot_msgs/local_setup.bash && \
    cmake -S /tmp/kyon_actions -B /tmp/kyon-actions-build \
      -DCMAKE_INSTALL_PREFIX=/opt/kyon-replay -DINSTALL_SIMULATION_TOOLS=OFF && \
    cmake --build /tmp/kyon-actions-build --parallel 4 && \
    cmake --install /tmp/kyon-actions-build && \
    ln -sf /workspace/kyon_actions/replayer_ros2.py \
      /workspace/kyon_actions/trajectory_io.py /opt/kyon-replay/lib/kyon_actions/ && \
    rm -rf /opt/kyon-replay/share/kyon_actions/config \
      /opt/kyon-replay/share/kyon_actions/mat_files && \
    ln -s /workspace/kyon_actions/config /workspace/kyon_actions/mat_files \
      /opt/kyon-replay/share/kyon_actions/

FROM ros:jazzy-ros-base
SHELL ["/bin/bash", "-c"]
ARG REPLAY_UID=1000
ARG REPLAY_GID=1000
RUN apt-get update && apt-get install -y --no-install-recommends \
      python3-numpy python3-scipy python3-yaml && \
    rm -rf /var/lib/apt/lists/* && \
    groupadd --gid "${REPLAY_GID}" --non-unique replay && \
    useradd --create-home --uid "${REPLAY_UID}" --gid replay --non-unique replay && \
    mkdir -p /data && chown replay:replay /data
COPY --from=build /opt/kyon-replay /opt/kyon-replay
COPY --chown=replay:replay . /workspace/kyon_actions
COPY --chmod=755 docker/entrypoint.sh /usr/local/bin/kyon-replay-entrypoint
ENV PYTHONDONTWRITEBYTECODE=1
USER replay
WORKDIR /workspace/kyon_actions
ENTRYPOINT ["/usr/local/bin/kyon-replay-entrypoint"]
CMD ["bash"]
