#May need to roll back to 2023.1.1

ARG CUDA_VERSION=12.2.0 ISAACSIM_VERSION=4.5.0 BASE_DIST=ubuntu20.04 CPU_PARALLEL=4

FROM nvcr.io/nvidia/isaac-sim:${ISAACSIM_VERSION} AS isaac-sim

FROM nvidia/cuda:${CUDA_VERSION}-devel-${BASE_DIST}

#TODO: Figure out what exactly makes the headless work in the isaac image
# Copy Isaac Sim files
COPY --from=isaac-sim / /

ENV DEBIAN_FRONTEND=noninteractive TZ=America/New_York \ 
    CMAKE_BUILD_PARALLEL_LEVEL=${CPU_PARALLEL} \
    NVIDIA_VISIBLE_DEVICES=all \
    NVIDIA_DRIVER_CAPABILITIES=all \
    CMAKE_BUILD_PARALLEL_LEVEL=${CPU_PARALLEL}

# Open ports for live streaming
EXPOSE 47995-48012/udp \
       47995-48012/tcp \
       49000-49007/udp \
       49000-49007/tcp \
       49100/tcp \
       8011/tcp \
       8012/tcp \
       8211/tcp \
       8899/tcp \
       8891/tcp


RUN mkdir -p /root/.nvidia-omniverse/config \
&& ln -s /isaac-sim/exts/omni.isaac.examples/omni/isaac/examples extension_examples 

#Python handling and quality of life
RUN echo 'alias python="bash /isaac-sim/python.sh"' >> ~/.bashrc \
&& echo 'alias python3="bash /isaac-sim/python.sh"' >> ~/.bashrc 
