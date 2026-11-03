# Python SDK / ROS client image, not a prebuilt HORUS bridge or MR application.
FROM ros:jazzy-ros-base-noble
RUN apt-get update && apt-get install -y --no-install-recommends python3-venv \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /opt/horus_sdk
COPY pyproject.toml README.md LICENSE ./
COPY python/horus ./python/horus
RUN /usr/bin/python3 -m venv --system-site-packages /opt/horus-python \
    && /opt/horus-python/bin/python -m pip install --no-cache-dir . \
    && /opt/horus-python/bin/python -c "import horus"
ENV PATH="/opt/horus-python/bin:${PATH}"
CMD ["bash"]
