#!/bin/bash

# Exit immediately if a command exits with a non-zero status
set -e

# Ensure the script is run with sudo
if [ "$EUID" -ne 0 ]; then
  echo "Please run this script with sudo."
  exit 1
fi

echo "🚀 Starting Post-Flash Xavier Setup for the Sub..."
REAL_USER=${SUDO_USER:-$USER}

# Get the absolute directory where this script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

# Dynamically find the parent directory (e.g., if script is in ~/sub_control_ws, this is ~/)
PARENT_DIR="$(dirname "$SCRIPT_DIR")"

# ==========================================
# 1. JETSON CONFIGURATION
# ==========================================
echo "Configuring hardware bus permissions..."
usermod -aG dialout,tty,i2c,gpio "$REAL_USER"
echo "User $REAL_USER added to dialout, tty, i2c, and gpio groups."

# Enable max Jetson clock speet on start up
# See details at https://nvidia-isaac-ros.github.io/v/release-2.1/getting_started/hardware_setup/compute/index.html
sudo /usr/bin/jetson_clocks
sudo /usr/sbin/nvpmodel -m 0

echo "Creating systemd service file at $SERVICE_FILE..."
SERVICE_FILE="/etc/systemd/system/jetson_clocks.service"

# Write the "setup jetson clocks" service configuration using a heredoc
cat << 'EOF' > "$SERVICE_FILE"
[Unit]
Description=Maximize Jetson Performance Clocks
After=nvpmodel.service
Requires=nvpmodel.service

[Service]
Type=oneshot
ExecStart=/usr/bin/jetson_clocks
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
EOF

echo "Service file created."

# Reload systemd to recognize the new jetson clocks file
echo "Reloading systemd daemon..."
systemctl daemon-reload

# Enable the service to run on boot
echo "Enabling jetson_clocks.service..."
systemctl enable jetson_clocks.service

# Start the service immediately
echo "Starting jetson_clocks.service..."
systemctl start jetson_clocks.service

# Verify it worked
echo "----------------------------------------"
if systemctl is-active --quiet jetson_clocks.service; then
    echo "Success: jetson_clocks.service is active!"
else
    echo "Warning: Service may not have started correctly. Run 'systemctl status jetson_clocks.service' to check."
fi
# TODO install jetson-stats

# ==========================================
# 2. XAVIER NVME MOUNTING (Safe Method)
# ==========================================
NVME_DEV="/dev/nvme0n1"
NVME_PART="/dev/nvme0n1p1"
MOUNT_POINT="/mnt/nvme"

echo "Setting up NVMe storage..."

if [ ! -b "$NVME_PART" ]; then
    echo "Partition $NVME_PART not found. Creating partition..."
    echo -e "n\np\n1\n\n\nw" | fdisk "$NVME_DEV"
    sleep 2 
fi

FS_TYPE=$(blkid -s TYPE -o value "$NVME_PART" || true)
if [ "$FS_TYPE" != "ext4" ]; then
    echo "No ext4 filesystem detected on $NVME_PART. Formatting now (this wipes data)..."
    mkfs.ext4 "$NVME_PART"
else
    echo "Existing ext4 filesystem found on $NVME_PART. Preserving data."
fi

mkdir -p "$MOUNT_POINT"

UUID=$(blkid -s UUID -o value "$NVME_PART")
if ! grep -q "$UUID" /etc/fstab; then
    echo "Adding NVMe to /etc/fstab for automounting..."
    echo "UUID=$UUID    $MOUNT_POINT    ext4    defaults    0    2" >> /etc/fstab
fi

mount -a
echo "NVMe mounted at $MOUNT_POINT."

# ==========================================
# 3. DOCKER CONFIGURATION
# ==========================================
echo "Configuring Docker..."

# Add Docker's official GPG key:
sudo apt-get update
sudo apt-get install ca-certificates curl gnupg -y
sudo install -m 0755 -d /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
sudo chmod a+r /etc/apt/keyrings/docker.gpg

# Add the repository to Apt sources:
echo \
"deb [arch="$(dpkg --print-architecture)" signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
"$(. /etc/os-release && echo "$VERSION_CODENAME")" stable" | \
sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt-get update
sudo apt install docker-buildx-plugin docker-compose-plugin -y

# Add default nvidia runtime to docker containers and move images to mounted NVMe
systemctl stop docker
mkdir -p "$MOUNT_POINT/docker"
cat <<EOF > /etc/docker/daemon.json
{
  "data-root": "$MOUNT_POINT/docker",
  "default-runtime": "nvidia",
  "runtimes": {
    "nvidia": {
      "path": "nvidia-container-runtime",
      "runtimeArgs": []
    }
  }
}
EOF

systemctl start docker
echo "Docker data-root updated to $MOUNT_POINT/docker."
usermod -aG docker "$REAL_USER"

# Refenrence to this https://nvidia-isaac-ros.github.io/v/release-2.1/getting_started/dev_env_setup.html
sudo apt-get install git-lfs
git lfs install --skip-repo

# ==========================================
# 4. DEPLOY SUB INFRASTRUCTURE
# ==========================================
echo "Deploying Sub container infrastructure..."

# Build ASUQTR ROS2 docker image (control/autonomy/hardware ros2 packages etc.)
docker build -t asuqtr_ros2:latest .

# Build ASUQTR Dashboard docker image
sudo -u "$REAL_USER" git clone "https://github.com/ASUQTR/dashboard.git" "~/asuqtr_dashboard"
cd ~/asuqtr_dashboard
docker build -t asuqtr-dashboard:latest .

# clone ASUQTR ROS2 vision workspace, to be mounted in ASUQTR vision container, which shall 
# be downloaded by docker compose
# HARDCODED, deterministic paths for the Sub
VISION_WS_DIR="/home/$REAL_USER/sub_vision_workspace"
VISION_REPO_URL="https://github.com/YOUR_ORG/sub_vision_workspace.git" # <-- Update this URL
echo "📂 Setting up the Vision Workspace at $VISION_WS_DIR..."

if [ ! -d "$VISION_WS_DIR" ]; then
    echo "📥 Cloning the Vision repository..."
    # Execute as the normal user so they retain ownership and SSH keys work
    sudo -u "$REAL_USER" git clone "$VISION_REPO_URL" "$VISION_WS_DIR"
else
    echo "✅ Vision workspace already exists. Pulling latest changes..."
    sudo -u "$REAL_USER" bash -c "cd $VISION_WS_DIR && git pull"
fi

# Docker Compose will automatically pull missing images from your registry
echo "📦 Pulling images and spinning up ASUQTR ROS2, ASUQTR dashboard and ASUQTR Vision ..."
docker compose -f "$SCRIPT_DIR/docker-compose.yaml" up -d

echo "=========================================="
echo "🎉 Setup Complete!"
echo "The Sub's control and vision containers are pulling/running in the background."
echo "Please log out and log back in (or reboot) for user group permissions to take effect."
echo "=========================================="
