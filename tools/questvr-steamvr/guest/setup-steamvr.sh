#!/bin/sh
Q=/root/.local/share/questdeck-vr; S=/root/.local/share/Steam; P=$Q/payload; L=$Q/logs
STATUS=$Q/setup.status
mkdir -p $Q/bin $Q/drivers $Q/config $L
: > $STATUS
say() { echo "$1|$2|$3" >> $STATUS; echo "[setup] $1: $2 $3"; }
console() { python3 $Q/bin/vrsteam-console.py "$@" > /dev/null 2>&1; }

say payload working
[ -d $P ] || { say payload failed "no payload (reinstall QuestFrame)"; exit 1; }
cp $P/run-steamvr.sh $P/vrsteam-proxy.py $Q/ && cp $P/vrfex $P/vrinit $P/vrsteam-console.py $P/vrlaunch.sh $P/vrcmd.sh $Q/bin/ && chmod 755 $Q/bin/* $Q/run-steamvr.sh || { say payload failed copy; exit 1; }
cp $P/questdeck-driver.tar $Q/questdeck-driver.tar
say payload done

FEX=$S/steamapps/common/FEX-Emu/usr/bin/FEX
sniper() { ls -d $S/steamapps/common/SteamLinuxRuntime_sniper/sniper_platform_*/files 2>/dev/null | tail -1; }
installed() { grep -q '"StateFlags"[[:space:]]*"4"' $S/steamapps/appmanifest_$1.acf 2>/dev/null; }
if ! installed 3127680 || ! installed 1628350; then
    say tools working "FEX and Steam Linux Runtime"
    console app_install 3127680; console app_install 1628350
    for i in $(seq 1 720); do installed 3127680 && installed 1628350 && break; sleep 5; done
    installed 3127680 && installed 1628350 || { say tools failed "Steam did not install them"; exit 1; }
    rm -f $Q/rootfs/.questframe-rootfs  # a new runtime: build the rootfs from it
fi
say tools done

V=$Q/SteamVR-linux
if [ ! -f $V/bin/linux64/vrserver ]; then
    say steamvr working "downloading SteamVR (about 6 GB)"
    for d in 250824 250823 250832; do console download_depot 250820 $d; done
    depot() { ls -d $S/steamapps/content/app_250820/depot_$1 $S/ubuntu12_32/steamapps/content/app_250820/depot_$1 2>/dev/null | head -1; }
    for i in $(seq 1 1440); do
        [ -n "$(depot 250824)" ] && [ -n "$(depot 250823)" ] && [ -n "$(depot 250832)" ] && \
            [ -z "$(pgrep -f 'steam.*download_depot' 2>/dev/null)" ] && [ -f "$(depot 250823)/bin/linux64/vrserver" ] && break
        sleep 5
    done
    [ -f "$(depot 250823)/bin/linux64/vrserver" ] || { say steamvr failed "download did not finish"; exit 1; }
    rm -rf $V.tmp; mkdir -p $V.tmp
    for d in 250824 250823 250832; do cp -a "$(depot $d)"/. $V.tmp/; done
    rm -rf $V; mv $V.tmp $V
fi
say steamvr done

D=$Q/rootfs
if [ ! -f $D/.questframe-rootfs ]; then
    say rootfs working
    python3 - "$(dirname "$(sniper)")" <<'PY' || { say rootfs failed links; exit 1; }
import importlib.machinery, importlib.util, sys
loader = importlib.machinery.SourceFileLoader("droiddeck_fex", "/usr/local/bin/droiddeck-fex")
spec = importlib.util.spec_from_loader(loader.name, loader)
fex = importlib.util.module_from_spec(spec)
loader.exec_module(fex)
fex.materialize(sys.argv[1])
PY
    rm -rf $D.tmp; cp -a "$(sniper)" $D.tmp || { say rootfs failed copy; exit 1; }
    for t in newlibs newlibs2; do
        rm -rf $Q/$t; mkdir -p $Q/$t && tar -xf $P/$t.tar -C $Q/$t || { say rootfs failed $t; exit 1; }
        for f in $Q/$t/*; do n=$(basename $f); rm -f $D.tmp/lib/x86_64-linux-gnu/$n; cp $f $D.tmp/lib/x86_64-linux-gnu/$n; chmod 755 $D.tmp/lib/x86_64-linux-gnu/$n; done
    done
    mkdir -p $D.tmp/lib64 && ln -sfn ../lib/x86_64-linux-gnu/ld-linux-x86-64.so.2 $D.tmp/lib64/ld-linux-x86-64.so.2 || { say rootfs failed loader; exit 1; }
    mkdir -p $D.tmp/root/.steam && ln -sfn $S/linux32 $D.tmp/root/.steam/sdk32 && ln -sfn $S/linux64 $D.tmp/root/.steam/sdk64
    touch $D.tmp/.questframe-rootfs
    rm -rf $D; mv $D.tmp $D
fi
say rootfs done

mkdir -p /root/.config/openvr
cat > /root/.config/openvr/openvrpaths.vrpath <<JSON
{ "config": ["$Q/config"], "external_drivers": ["$Q/drivers/questdeck"], "jsonid": "vrpathreg", "log": ["$L"], "runtime": ["$V"], "version": 1 }
JSON
say ready done
