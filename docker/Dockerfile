# syntax=docker/dockerfile:1
ARG CONTROLLER_IMAGE=kyon-controller:latest
FROM ${CONTROLLER_IMAGE}

SHELL ["/bin/bash", "-c"]
WORKDIR /home/user/kyon_ws

# Include the local ROS 2 changes and trajectories, including uncommitted files.
COPY --chown=user:user src/kyon_controller/ src/kyon_controller/
COPY --chown=user:user src/kyon_actions/ src/kyon_actions/

RUN source /opt/ros/jazzy/setup.bash && \
    source /opt/xbot/setup.sh && \
    source /home/user/env/bin/activate && \
    source /home/user/kyon_ws/setup.bash && \
    cmake -S src/kyon_controller -B /tmp/kyon-controller-replay-build \
      -DCMAKE_INSTALL_PREFIX=/home/user/kyon_ws/install \
      -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF && \
    cmake --build /tmp/kyon-controller-replay-build --parallel 4 && \
    cmake --install /tmp/kyon-controller-replay-build && \
    cmake -S src/kyon_actions -B /tmp/kyon-actions-replay-build \
      -DCMAKE_INSTALL_PREFIX=/home/user/kyon_ws/install -DBUILD_TESTING=OFF && \
    cmake --build /tmp/kyon-actions-replay-build --parallel 4 && \
    cmake --install /tmp/kyon-actions-replay-build && \
    rm -rf /tmp/kyon-controller-replay-build /tmp/kyon-actions-replay-build

COPY --chmod=755 src/kyon_actions/docker/entrypoint.sh /usr/local/bin/kyon-replay-entrypoint
ENV PYTHONDONTWRITEBYTECODE=1
WORKDIR /data
ENTRYPOINT ["/usr/local/bin/kyon-replay-entrypoint"]
CMD ["bash"]
