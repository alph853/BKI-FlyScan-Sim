#!/bin/sh

# Configure hotspot using NetworkManager
nmcli connection delete "UAV-JETSON-CONFIG" 2>/dev/null

# Create hotspot
nmcli connection add \
    type wifi \
    ifname wlan0 \
    con-name "UAV-JETSON-CONFIG" \
    autoconnect no \
    ssid "UAV-JETSON-CONFIG"

nmcli connection modify "UAV-JETSON-CONFIG" \
    802-11-wireless.mode ap \
    802-11-wireless.band bg \
    ipv4.method shared \
    ipv4.addresses 192.168.4.1/24 \
    wifi-sec.key-mgmt wpa-psk \
    wifi-sec.psk "uavpilots"

# Start hotspot
nmcli connection up "UAV-JETSON-CONFIG"

# Configure dnsmasq for captive portal
cat > /etc/dnsmasq.conf << EOF
interface=wlan0
bind-interfaces
server=8.8.8.8
domain-needed
bogus-priv
dhcp-range=192.168.4.2,192.168.4.100,12h
address=/#/192.168.4.1
EOF

# Restart dnsmasq
killall dnsmasq 2>/dev/null
dnsmasq

# Configure iptables for captive portal
iptables -t nat -A PREROUTING -i wlan0 -p tcp --dport 80 -j DNAT --to-destination 192.168.4.1:80
iptables -t nat -A PREROUTING -i wlan0 -p tcp --dport 443 -j DNAT --to-destination 192.168.4.1:80
