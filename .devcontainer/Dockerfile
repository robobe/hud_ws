FROM humble:dev

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-opencv \
    ros-humble-vision-msgs \
    ros-humble-cv-bridge \
  && rm -rf /var/lib/apt/lists/*