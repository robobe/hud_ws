FROM humble:dev

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-opencv \
    ros-humble-cv-bridge \
    ros-humble-vision-msgs \
  && rm -rf /var/lib/apt/lists/*