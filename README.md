# Crazyflie Simulation Project

This repository contains shared Python scripts, Crazyflie simulations, Gazebo worlds, configuration files, and project documentation.

Each team member uses:

* a local Docker image compatible with their computer;
* the same GitHub repository;
* a platform-specific Docker Compose configuration for the Gazebo graphical interface.

The repository is mounted inside the Docker container at:

```text
/workspace/crazyflie-project
```

Changes made on the host computer are immediately visible inside the container.

---

## Project Structure

```text
crazyflie-project/
├── README.md
├── .gitignore
├── .env.example
├── compose.yaml
├── compose.ubuntu.yaml
├── compose.macos.yaml
├── requirements.txt
├── scripts/
├── src/
├── configs/
├── worlds/
├── models/
└── outputs/
```

---

## 1. Clone the Repository

```bash
git clone https://github.com/YOUR_GITHUB_USERNAME/crazyflie-project.git
cd crazyflie-project
```

Replace `YOUR_GITHUB_USERNAME` with the actual GitHub username or organization name.

---

## 2. Configure the Local Docker Image

List the Docker images available on your computer:

```bash
docker image ls
```

The image name is composed of:

```text
REPOSITORY:TAG
```

For example:

```text
crazysim-local:latest
```

Create the local environment file:

```bash
cp .env.example .env
```

The shared `.env.example` file must contain:

```env
CRAZYSIM_IMAGE=your-image-name:tag
```

Edit the local `.env` file and insert the name of your Docker image.

Ubuntu example:

```env
CRAZYSIM_IMAGE=crazysim-ubuntu:latest
```

macOS example:

```env
CRAZYSIM_IMAGE=crazysim-macos:latest
```

The `.env` file is local and must not be committed to GitHub.

---

## 3. Common Docker Compose Configuration

The shared `compose.yaml` file contains the configuration used by all team members.

```yaml
services:
  crazysim:
    image: ${CRAZYSIM_IMAGE}

    working_dir: /workspace/crazyflie-project

    volumes:
      - .:/workspace/crazyflie-project

    stdin_open: true
    tty: true
```

---

## 4. Ubuntu Graphical Configuration

Create the file:

```text
compose.ubuntu.yaml
```

with the following content:

```yaml
services:
  crazysim:
    environment:
      DISPLAY: ${DISPLAY}
      QT_X11_NO_MITSHM: "1"

    volumes:
      - /tmp/.X11-unix:/tmp/.X11-unix:rw
```

Before starting the container, allow Docker to access the X11 display:

```bash
xhost +local:docker
```

Start the container with:

```bash
docker compose \
  -f compose.yaml \
  -f compose.ubuntu.yaml \
  run --rm crazysim bash
```

Inside the container, run the required CrazySim, Gazebo, or Python commands.

When finished, exit the container:

```bash
exit
```

Optionally remove the X11 permission:

```bash
xhost -local:docker
```

---

## 5. macOS Configuration with XQuartz

macOS users must install:

* Docker Desktop for Mac;
* XQuartz.

After installing XQuartz:

1. Open XQuartz.
2. Open:

```text
XQuartz → Settings → Security
```

3. Enable:

```text
Allow connections from network clients
```

4. Close and restart XQuartz.

Create the file:

```text
compose.macos.yaml
```

with the following content:

```yaml
services:
  crazysim:
    environment:
      DISPLAY: host.docker.internal:0
      QT_X11_NO_MITSHM: "1"
```

Do not mount `/tmp/.X11-unix` on macOS.

Docker Desktop runs containers inside a Linux virtual machine, so the container connects to XQuartz through:

```text
host.docker.internal
```

Before starting Docker, open XQuartz.

Then run the following command from a macOS terminal:

```bash
/opt/X11/bin/xhost +localhost
```

Start the container with:

```bash
docker compose \
  -f compose.yaml \
  -f compose.macos.yaml \
  run --rm crazysim bash
```

Inside the container, run the required CrazySim, Gazebo, or Python commands.

When Gazebo starts, its graphical window should appear through XQuartz.

When finished, exit the container:

```bash
exit
```

Optionally remove the XQuartz permission:

```bash
/opt/X11/bin/xhost -localhost
```

---

## 6. Verify the Shared Repository

After entering the container, check the current directory:

```bash
pwd
```

The expected result is:

```text
/workspace/crazyflie-project
```

List the project files:

```bash
ls
```

Run the environment test:

```bash
python3 scripts/check_environment.py
```

The script should confirm that the repository is correctly mounted inside Docker.

---

## 7. Edit Files from the Host Computer

Project files should be edited on the host computer, not inside the container.

For example:

```bash
nano scripts/example.py
```

or:

```bash
code .
```

Because the repository is mounted inside Docker, saved changes are immediately visible inside the container.

If an editor such as `nano` is not installed inside Docker, exit the container:

```bash
exit
```

Edit the file on the host computer and restart the container.

---

## 10. Quick Start

### Ubuntu

```bash
git clone https://github.com/YOUR_GITHUB_USERNAME/crazyflie-project.git
cd crazyflie-project

cp .env.example .env
nano .env

xhost +local:docker

docker compose \
  -f compose.yaml \
  -f compose.ubuntu.yaml \
  run --rm crazysim bash
```

### macOS

```bash
git clone https://github.com/YOUR_GITHUB_USERNAME/crazyflie-project.git
cd crazyflie-project

cp .env.example .env
nano .env
```

Open XQuartz and enable:

```text
Allow connections from network clients
```

Then run:

```bash
/opt/X11/bin/xhost +localhost

docker compose \
  -f compose.yaml \
  -f compose.macos.yaml \
  run --rm crazysim bash
```
