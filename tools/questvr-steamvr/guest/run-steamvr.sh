#!/bin/sh
V=/root/.local/share/questdeck-vr/SteamVR-linux; Q=/root/.local/share/questdeck-vr; L=$Q/logs; CFG=$Q/config
S=/root/.local/share/Steam
pgrep -f "python3 $Q/vrsteam-proxy.py" > /dev/null || setsid python3 $Q/vrsteam-proxy.py > $L/vrsteam-proxy.log 2>&1 < /dev/null &
if pgrep -f "vrserver --keepalive" > /dev/null && pgrep -f "linux64/vrcompositor" > /dev/null; then echo "SteamVR already running"; exit 0; fi
[ -f $Q/questdeck-driver.tar ] && rm -rf $Q/drivers/questdeck && tar -xf $Q/questdeck-driver.tar -C $Q/drivers
pkill -9 -f "SteamVR-linux/bin/" ; pkill -9 -f "vrserver --keepalive"; pkill -9 -f "questdeck-vr/bin/vrinit"; pkill -9 -f vrwebhelper
for p in $(pgrep -f FEXServer); do tr '\0' '\n' < /proc/$p/environ 2>/dev/null | grep -q fex-questdeck-vr && kill -9 $p; done
sleep 1
rm -f /dev/shm/u*-ValveIPCSharedObj-SteamVR /dev/shm/u*-Shm_* /dev/shm/u*-VRWebHelper* /dev/shm/qd-sysvsem-* 2>/dev/null
rm -f $L/*.txt

mkdir -p $Q/rootfs/root/.steam
mkdir -p $S/steamapps/compatdata/250820/fex-emu/Server
ln -sfn $S/linux64 $Q/rootfs/root/.steam/sdk64; ln -sfn $S/linux32 $Q/rootfs/root/.steam/sdk32

W=$V/bin/vrwebhelper/linux64/vrwebhelper.sh
[ -f $W.orig ] || cp $W $W.orig
python3 - "$W" <<'P'
import sys
p = sys.argv[1]; s = open(p + ".orig").read()
old = 'exec "${in_runtime[@]}" ./vrwebhelper "$@"'
s = s.replace(old, old + ' --no-zygote --disable-gpu --disable-dev-shm-usage')
open(p, "w").write(s)
P

cat > $CFG/steamvr.vrsettings <<J
{
  "steamvr": { "requireHmd": true, "forcedDriver": "questdeck", "activateMultipleDrivers": false,
               "enableHomeApp": false, "showMirrorView": false, "startMonitorFromAppLaunch": false,
               "startCompositorFromAppLaunch": true, "startDashboardFromAppLaunch": true },
  "driver_questdeck": { "enable": true },
  "driver_null": { "enable": false }, "driver_lighthouse": { "enable": false }, "driver_vrlink": { "enable": false }
}
J

cd "$V/bin/linux64"
export LD_LIBRARY_PATH="$V/bin/linux64:$S/linux64:$Q/lib64"
export SYSTEM_LD_LIBRARY_PATH="$LD_LIBRARY_PATH" VALVE_SKIP_RUNTIME_SAFETY=1 DISABLE_GAMESCOPE_WSI=1
unset ENABLE_GAMESCOPE_WSI
export LD_PRELOAD=$Q/drivers/questdeck/layer/libquestdeck_kgslts.so:$Q/drivers/questdeck/layer/libquestdeck_vrenv.so
export QUESTDECK_EYE_IMPORT=ubwc
export VK_ADD_LAYER_PATH=$Q/drivers/questdeck/layer VK_INSTANCE_LAYERS=VK_LAYER_QUESTDECK_steamvr
T=${1:-1800}
timeout -s KILL $T $Q/bin/vrfex ./vrserver --keepalive > $L/out2.txt 2>&1 &
sleep 15
timeout -s KILL 30 $Q/bin/vrfex $Q/bin/vrinit 1 2 > /dev/null 2>&1; sleep 10
timeout -s KILL $T $Q/bin/vrfex $Q/bin/vrinit 1 $T > $L/vrinit.out 2>&1 &
sleep 60
timeout -s KILL 40 $Q/bin/vrfex ./vrcmd --showdashboard > /dev/null 2>&1
grep -h "questdeck\]" $L/out2.txt | tail -3
