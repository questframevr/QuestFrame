# SteamVR guest files

Scripts and helpers that run inside the Linux guest for SteamVR mode.

- `setup-steamvr.sh` installs everything SteamVR needs into `/root/.local/share/questdeck-vr`
- `run-steamvr.sh` starts SteamVR
- `vrsteam-proxy.py` connects the Steam client to SteamVR and fills the VR library
- `questframe-proton-vr/` is the Proton tool for VR games

Build the native parts with `../build.sh`.
