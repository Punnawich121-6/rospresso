#!/bin/bash

sudo apt update

echo "Installing PipeWire and audio libraries..."
sudo apt install -y pipewire pipewire-audio-client-libraries pipewire-pulse wireplumber libspa-0.2-bluetooth

echo "Disabling PulseAudio..."
systemctl --user --now disable pulseaudio.service pulseaudio.socket
systemctl --user mask pulseaudio

echo "Enabling PipeWire services..."
systemctl --user --now enable pipewire
systemctl --user --now enable pipewire-pulse

echo "Installing audio control tools..."
sudo apt install -y pavucontrol blueman

echo "Restarting PipeWire for good measure..."
systemctl --user restart pipewire pipewire-pulse wireplumber

echo ""
echo "PipeWire is now set up and running!"
echo "Available audio sinks (Bluetooth devices usually start with bluez_...):"
pactl list short sinks

echo ""
echo "If your Bluetooth speaker appears above, you can set it as the default with:"
echo "pactl set-default-sink <sink-name>"
echo "Example:"
echo "pactl set-default-sink bluez_sink.XX_XX_XX_XX_XX_XX.a2dp_sink"

echo ""
echo "Done! You can now open 'pavucontrol' to manage audio routing and 'blueman-manager' to auto-connect your speaker."
