---
title: Troubleshooting
sidebar_position: 1
---

# Troubleshooting

## No ACK after registration

Check:

```bash
ros2 topic echo /horus/registration_ack
ros2 topic list | grep horus
```

If silent, verify backend launch and app connection lifecycle.

## Dashboard false states

Validate sequence:

1. app disconnected -> `IDLE`
2. app connected, pre-workspace -> not falsely `ACTIVE`
3. workspace accepted + publishers active -> `ACTIVE`

## Humble installer warnings

If install reports backend-only mode, this is expected with current bridge header constraints.

Use Jazzy for full bridge builds:

```bash
bash install.sh --yes --ros-distro jazzy --webrtc on
```

## WSL2 + Meta Quest connectivity

Apply mirrored networking in `%USERPROFILE%\\.wslconfig`:

```ini
[wsl2]
networkingMode=mirrored
```

Then:

```powershell
wsl --shutdown
```
