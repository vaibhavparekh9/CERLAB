# Isaac Sandbox

## Recommendations Before You Get Started
The main requirement to begin developing with Nvidia Isaac Sim is knowledge of Python programming. We also recommend some basic knowledge of using the terminal in Ubuntu. You can find a good tutorial on using the Ubuntu terminal [here](https://ubuntu.com/tutorials/command-line-for-beginners#3-opening-a-terminal).

### Container Registry
This container is hosted on the GitLab Container Registry at `gitlab.cerlab.lan.cmu.edu:5050`.

#### Login to the Registry
Open a terminal and run the following command to log in to the GitLab Container Registry. You will need your GitLab username and password.

```bash
docker login gitlab.cerlab.lan.cmu.edu:5050
```
#### Pull docker image
To pull the docker image run 

```bash
docker pull gitlab.cerlab.lan.cmu.edu:5050/cerlab_public/isaac-sandbox
```


#### Overwrite the Default Password
To overwrite the default Docker Nucleus account login to be able to save .usd files, create a file named `password.env` in the `./docker` folder of your project. Add your desired password to this file in the following format:

```plaintext
OMNI_USER=your_nucleus_username
OMNI_PASS=your_nucleus_password
```

Make sure to replace `your_nucleus_password` with your Nucleus server password (not necessarily your GitLab password). Please contact the local server administrator if you have any issues with your password. Save the file and ensure it is included in your `.gitignore` file to keep your password secure.

### Starting the Container and Running Isaac Sim
This repository is meant to provide an environment that is easy to set up and use for learning Nvidia Isaac Sim. To run Isaac, navigate to the Docker directory and start the Docker container with the command `./run_docker.sh`. Once the container is loaded, use the command `./open_isaac.sh` to open Isaac Sim. We recommend starting with the Isaac Sim tutorials found [here](https://docs.omniverse.nvidia.com/isaacsim/latest/introductory_tutorials/tutorial_intro_interface.html).

### Extending the Base Image

This Docker image is intended as a starting point for working with Nvidia Isaac Sim. If you need to add more dependencies or customize the environment further, you can use this image as a base layer in your own Dockerfile. Here is an example of how to do this:

```Dockerfile
FROM gitlab.cerlab.lan.cmu.edu:5050/isaac-sandbox:latest

# Install additional dependencies
RUN apt-get update && apt-get install -y \
    some-package \
    another-package

# Copy your project files
COPY . /path/to/your/project

# Set the working directory
WORKDIR /path/to/your/project

# Run any additional setup commands
RUN ./setup_script.sh

# Define the command to run your application
CMD ["./start_application.sh"]
```

### Docker Compose Configuration

The settings in the provided `docker-compose.yml` file allow your system to properly interface with the GUI of Nvidia Isaac Sim, and it is necessary to use `docker-compose up` to run this image. This configuration ensures that necessary volumes, environment variables, and GPU capabilities are correctly set up.

Feel free to use this file as a template if any changes are desired to fit your specific requirements. If you encounter any issues or need assistance in making necessary changes, please reach out at gmetts@andrew.cmu.edu.

### Important Notice

This Docker deployment is designed to provide a Dockerized graphical user interface (GUI) for Nvidia Isaac Sim for easy deployment within CERLAB. Please note that Nvidia Isaac Sim and all related assets fall under Nvidia's intellectual property (IP) rights. Ensure that you comply with all applicable licensing agreements and terms of use when utilizing this software.