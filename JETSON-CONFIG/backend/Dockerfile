FROM python:3.9-alpine

RUN apk add --no-cache \
    networkmanager \
    networkmanager-wifi \
    dbus \
    iw \
    wireless-tools \
    hostapd \
    dnsmasq \
    iptables

WORKDIR /app

COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

COPY . .

EXPOSE 80