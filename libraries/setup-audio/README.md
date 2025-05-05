# 🔊 Bluetooth Audio Setup for Ubuntu 22.04 (PipeWire Version)

This script helps you set up high-quality, stable Bluetooth audio on Ubuntu 22.04 by:

- Replacing PulseAudio with PipeWire
- Enabling A2DP high-fidelity audio profile
- Installing tools to manage audio sinks and Bluetooth pairing
- Ensuring Chrome, Firefox, and other apps route audio correctly to your Bluetooth speaker

---

## ✅ Features

- PipeWire and Bluetooth audio module installation
- PulseAudio safe disable + replacement with PipeWire
- Auto-enable PipeWire services
- Includes GUI tools: `pavucontrol` (volume control) and `blueman` (Bluetooth manager)
- Displays available audio sinks for manual default setting

---

## ⚙️ How to Use

### 1. Clone or download this script:
Or simply create the file manually.

```bash
nano setup-bluetooth-pipewire.sh
