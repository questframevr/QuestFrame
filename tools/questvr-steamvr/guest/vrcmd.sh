#!/bin/sh
Q=/root/.local/share/questdeck-vr; V=$Q/SteamVR-linux; S=/root/.local/share/Steam
cd "$V/bin/linux64" || exit 1
export LD_LIBRARY_PATH="$V/bin/linux64:$S/linux64:$Q/lib64"
export SYSTEM_LD_LIBRARY_PATH="$LD_LIBRARY_PATH" VALVE_SKIP_RUNTIME_SAFETY=1
export LD_PRELOAD=$Q/drivers/questdeck/layer/libquestdeck_vrenv.so
exec timeout -s KILL 60 $Q/bin/vrfex ./vrcmd "$@"
