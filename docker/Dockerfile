# Use the carlasim/carla:0.9.15 image as the base
FROM carlasim/carla:0.9.15

# download scenario runner
RUN python3 -c "import urllib.request; \
	urllib.request.urlretrieve( \
	'https://github.com/carla-simulator/scenario_runner/archive/refs/tags/v0.9.15.tar.gz',\ 
	'scenario_runner0.9.15.tar.gz')"
RUN tar -xf scenario_runner0.9.15.tar.gz

# set environment variables for scenario runner
ENV CARLA_ROOT=/home/carla
ENV PYTHONPATH=${CARLA_ROOT}/PythonAPI/carla/dist/carla-0.9.15-py3.7-linux-x86_64.egg:${CARLA_ROOT}/PythonAPI/carla/agents:${CARLA_ROOT}/PythonAPI/carla

# install Anaconda into container
RUN python3 -c "import urllib.request; \
	urllib.request.urlretrieve( \
	'https://repo.anaconda.com/archive/Anaconda3-2024.06-1-Linux-x86_64.sh', \
	'get-anaconda.sh')"
RUN /bin/bash get-anaconda.sh -b

# create scenario-runner environment and install packages into it
RUN /home/carla/anaconda3/bin/conda create --name scenario-runner-env python=3.7 -y
RUN /home/carla/anaconda3/bin/conda run -n scenario-runner-env \
	pip install -r scenario_runner-0.9.15/requirements.txt

# install further scenario-runner dependencies (shared libraries)
#switch to root user temporarily
USER root 
RUN apt-get install -y libjpeg-dev
RUN apt-get install -y libtiff-dev
#switch back to designated user from carlasim/carla:0.9.15
USER carla 


# initialize conda
RUN /home/carla/anaconda3/bin/conda init
