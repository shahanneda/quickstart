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
sudo sed -i 's/^use-ipv4=.*/use-ipv4=yes/' /etc/avahi/avahi-daemon.conf
sudo sed -i 's/^use-ipv6=.*/use-ipv6=no/' /etc/avahi/avahi-daemon.conf

# Add allowed interfaces to avahi-daemon.conf
if grep -q '^allow-interfaces=' /etc/avahi/avahi-daemon.conf; then
    sudo sed -i 's/^allow-interfaces=.*/allow-interfaces=eth0,wlan0,wlan0_ap/' /etc/avahi/avahi-daemon.conf
else
    echo "allow-interfaces=eth0,wlan0,wlan0_ap" | sudo tee -a /etc/avahi/avahi-daemon.conf
fi

# Restart avahi-daemon
sudo systemctl restart avahi-daemon

# Configure NetworkManager
if ! grep -q '^\[connection\]' /etc/NetworkManager/NetworkManager.conf; then
    echo -e "\n[connection]" | sudo tee -a /etc/NetworkManager/NetworkManager.conf
fi
sudo sed -i '/^\[connection\]/,/^\[/{s/^ipv4\.mdns=.*/ipv4.mdns=2/;t;s/^ipv6\.mdns=.*/ipv6.mdns=2/;t}' /etc/NetworkManager/NetworkManager.conf
if ! grep -q '^ipv4\.mdns=2' /etc/NetworkManager/NetworkManager.conf; then
    sudo sed -i '/^\[connection\]/a ipv4.mdns=2' /etc/NetworkManager/NetworkManager.conf
fi
if ! grep -q '^ipv6\.mdns=2' /etc/NetworkManager/NetworkManager.conf; then
    sudo sed -i '/^\[connection\]/a ipv6.mdns=2' /etc/NetworkManager/NetworkManager.conf
fi

# Restart NetworkManager
sudo systemctl restart NetworkManager

# Create a Wi-Fi hotspot connection
sudo nmcli connection add type wifi ifname wlan0_ap con-name MyHotspot autoconnect yes ssid "$SSID"

# Configure the hotspot settings
sudo nmcli connection modify MyHotspot 802-11-wireless.mode ap 802-11-wireless.band bg ipv4.method shared

# Set the Wi-Fi security (WPA-PSK)
sudo nmcli connection modify MyHotspot wifi-sec.key-mgmt wpa-psk wifi-sec.psk "$PASSWORD"

# Bring up the hotspot
sudo nmcli connection up MyHotspot

# Ensure SSH service is enabled and running
sudo systemctl enable ssh
sudo systemctl start ssh

echo "Setup complete."
echo "The access point '$SSID' is now active with password '$PASSWORD'."
echo "You can SSH into the Raspberry Pi using its hostname over the access point network."
