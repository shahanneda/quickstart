#!/bin/bash

# Exit immediately if a command exits with a non-zero status
set -e

# Get the current logged-in username
USERNAME=$(whoami)

# Define the SSID and password
SSID="${USERNAME}-bracketbot"
PASSWORD="12345678"

echo "Setting up Wi-Fi access point with SSID: $SSID"

# Update system packages
sudo apt update

# Install necessary packages
sudo apt install -y avahi-daemon avahi-utils net-tools

# Enable and start avahi-daemon
sudo systemctl enable avahi-daemon
sudo systemctl start avahi-daemon

# Configure avahi-daemon
AVAHI_CONF="/etc/avahi/avahi-daemon.conf"

# Replace or add use-ipv4 and use-ipv6 in [server] section
sudo sed -i '/^\[server\]/,/^\[.*\]/ s/^use-ipv4=.*/use-ipv4=yes/' "$AVAHI_CONF"
sudo sed -i '/^\[server\]/,/^\[.*\]/ s/^use-ipv6=.*/use-ipv6=no/' "$AVAHI_CONF"

# Add or update allow-interfaces in [server] section
if grep -q '^\[server\]' "$AVAHI_CONF"; then
    # Check if allow-interfaces is already set
    if grep -A5 '^\[server\]' "$AVAHI_CONF" | grep -q '^allow-interfaces='; then
        # Update existing allow-interfaces line
        sudo sed -i '/^\[server\]/,/^\[.*\]/ s/^allow-interfaces=.*/allow-interfaces=eth0,wlan0,wlan0_ap/' "$AVAHI_CONF"
    else
        # Add allow-interfaces line after [server] section
        sudo sed -i '/^\[server\]/a allow-interfaces=eth0,wlan0,wlan0_ap' "$AVAHI_CONF"
    fi
else
    # [server] section not found, add it to the top of the file
    echo -e "[server]\nallow-interfaces=eth0,wlan0,wlan0_ap" | sudo tee -a "$AVAHI_CONF"
fi

# Restart avahi-daemon
sudo systemctl restart avahi-daemon

# Configure NetworkManager
NM_CONF="/etc/NetworkManager/NetworkManager.conf"

# Append connection settings to NetworkManager config
echo -e "\n[connection]\nipv4.mdns=2\nipv6.mdns=2" | sudo tee -a "$NM_CONF"

# Restart NetworkManager
sudo systemctl restart NetworkManager

# Create virtual AP interface
sudo iw dev wlan0 interface add wlan0_ap type __ap

# Create a Wi-Fi hotspot connection
sudo nmcli connection add type wifi ifname wlan0_ap con-name MyHotspot autoconnect yes ssid "$SSID"

# Configure the hotspot settings
sudo nmcli connection modify MyHotspot 802-11-wireless.mode ap 802-11-wireless.band bg ipv4.method shared

# Set the Wi-Fi security (WPA-PSK)
sudo nmcli connection modify MyHotspot wifi-sec.key-mgmt wpa-psk wifi-sec.psk "$PASSWORD"

# Bring up both interfaces
sudo nmcli connection up MyHotspot
# sudo nmcli device connect wlan0

# Ensure SSH service is enabled and running
if ! systemctl is-active ssh >/dev/null 2>&1; then
    sudo systemctl enable ssh
    sudo systemctl start ssh
else
    sudo systemctl restart ssh
fi

echo "Setup complete."
echo "The access point '$SSID' is now active with password '$PASSWORD'."
echo "You can SSH into the Raspberry Pi using its hostname over the access point network."
