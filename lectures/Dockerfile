FROM ros:jazzy-ros-base

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND="noninteractive"
ENV TZ="Europe/Paris"

WORKDIR /deps

RUN apt-get update && apt-get upgrade -y && apt-get clean  && apt-get install -y libboost-all-dev terminator gedit locate cmake-curses-gui python3-pip python3-venv liburdfdom-dev git-all libeigen3-dev libhdf5-dev \ 
    ros-jazzy-urdf \
    ros-jazzy-urdfdom \
    ros-jazzy-urdfdom-headers \
    ros-jazzy-srdfdom \
    ros-jazzy-geometric-shapes \
    ros-jazzy-moveit-msgs
    
WORKDIR /home
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc

# create forest ws and use it to clone and install CONCERT's simulation package
WORKDIR /home/forest_ws
ENV HHCM_FOREST_CLONE_DEFAULT_PROTO=https
ENV PYTHONUNBUFFERED=1

# create python virtual env
RUN python3 -m venv /home/.base
RUN echo "source /home/.base/bin/activate" >> ~/.bashrc
SHELL ["bash", "-ic"]

RUN echo "export CMAKE_PREFIX_PATH=/home/forest_ws/install:$CMAKE_PREFIX_PATH" >> ~/.bashrc
SHELL ["bash", "-ic"]


# MatLogger2
WORKDIR /home/forest_ws/src
RUN git clone -b master https://github.com/hucebot/MatLogger2.git && \
    cd /home/forest_ws/src/MatLogger2 && \
    git checkout 21f9581a8c01f92609f4ec20d0d32db30d4194ba && \
    mkdir -p /home/forest_ws/build/MatLogger2 && \
    cd /home/forest_ws/build/MatLogger2 && \
    source /opt/ros/jazzy/setup.bash && \
    cmake -DCMAKE_INSTALL_PREFIX:STRING=/home/forest_ws/install -DPYBIND11_PYTHON_VERSION=3 -DPYTHON_VERSION=3 -DCMAKE_BUILD_TYPE:STRING=Release ../../src/MatLogger2 && \
    make -j8 && \
    make install
    
# COAL
WORKDIR /home/forest_ws/src
RUN git clone -b devel https://github.com/coal-library/coal.git && \
    cd /home/forest_ws/src/coal && \
    git checkout 9407d538c36bf484c9b18d8d4397d09722698fc4 && \
    git submodule init && \
    git submodule update && \
    mkdir -p /home/forest_ws/build/coal && \
    cd /home/forest_ws/build/coal && \
    source /opt/ros/jazzy/setup.bash && \
    cmake -DCMAKE_INSTALL_PREFIX:STRING=/home/forest_ws/install -DCMAKE_BUILD_TYPE:STRING=Release -DBUILD_PYTHON_INTERFACE=OFF ../../src/coal && \
    make -j8 && \
    make install

## PINOCCHIO
WORKDIR /home/forest_ws/src
RUN git clone -b devel https://github.com/stack-of-tasks/pinocchio.git && \
    cd /home/forest_ws/src/pinocchio && \
    git checkout 4a0e2c40e6991173a40e587e1adff7f974172df3 && \
    git submodule init && \
    git submodule update && \
    mkdir -p /home/forest_ws/build/pinocchio && \
    cd /home/forest_ws/build/pinocchio && \
    source /opt/ros/jazzy/setup.bash && \
    cmake -DCMAKE_INSTALL_PREFIX:STRING=/home/forest_ws/install -DCMAKE_BUILD_TYPE:STRING=Release -DBUILD_WITH_URDF_SUPPORT=ON -DBUILD_WITH_COLLISION_SUPPORT=OFF -DBUILD_TESTING=FALSE -DBUILD_PYTHON_INTERFACE=OFF ../../src/pinocchio && \
    make -j8 && \
    make install
    

# xbot2_interface
WORKDIR /home/forest_ws/src
RUN git clone -b devel https://github.com/hucebot/xbot2_interface.git && \   
    cd /home/forest_ws/src/xbot2_interface && \
    git checkout 0d688b7bc3c3794b6d7670354b6bd3777b9cc6f1 && \
    mkdir -p /home/forest_ws/build/xbot2_interface && \
    cd /home/forest_ws/build/xbot2_interface && \
    source /opt/ros/jazzy/setup.bash && \
    cmake -DXBOT2_IFC_BUILD_TESTS=ON -DXBOT2_IFC_BUILD_ROS=OFF -DXBOT2_IFC_BUILD_ROS2=OFF -DXBOT2_IFC_BUILD_COLLISION=ON -DCMAKE_INSTALL_PREFIX:STRING=/home/forest_ws/install -DCMAKE_BUILD_TYPE:STRING=Release ../../src/xbot2_interface && \
    make -j8 && \
    make install
    
# OpenSoT
WORKDIR /home/forest_ws/src
RUN git clone -b ros2 https://github.com/hucebot/OpenSoT.git && \   
    cd /home/forest_ws/src/OpenSoT && \
    git checkout f2d9cd943131b9d8ffd3a5f3442119473e75bc62 && \
    mkdir -p /home/forest_ws/build/OpenSoT && \
    cd /home/forest_ws/build/OpenSoT && \
    source /opt/ros/jazzy/setup.bash && \
    cmake -DOPENSOT_COMPILE_COLLISION=ON -DOPENSOT_COMPILE_OSQP_BACKEND=OFF -DCMAKE_INSTALL_PREFIX:STRING=/home/forest_ws/install -DCMAKE_BUILD_TYPE:STRING=Release ../../src/OpenSoT && \
    make -j8 && \
    make install
    
    
RUN apt-get update && apt-get upgrade -y && apt-get clean  && apt-get install -y libxcb-cursor0 gdb 
RUN pip install viser open3d sympy jupyter importnb

RUN echo "export PYTHONPATH=${PYTHONPATH}:/root/.local/lib/python3.12/site-packages:/usr/lib/python3/dist-packages/:/home/forest_ws/install/lib/python3.12/site-packages/" >> ~/.bashrc
RUN echo "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/forest_ws/install/lib/" >> ~/.bashrc
RUN echo "export XDG_RUNTIME_DIR=/tmp/runtime-$USER" >> ~/.bashrc

WORKDIR /home 

