R=/root/.local/share/Steam/steamapps/common/SteamLinuxRuntime_sniper/sniper_platform_3.0.20260805.254768/files
Q=/root/.local/share/questdeck-vr; D=$Q/rootfs
rm -rf $D.tmp; cp -a $R $D.tmp || exit 1
mkdir -p $Q/newlibs && tar -xf $Q/newlibs.tar -C $Q/newlibs
for f in $Q/newlibs/*; do n=$(basename $f); rm -f $D.tmp/lib/x86_64-linux-gnu/$n; cp $f $D.tmp/lib/x86_64-linux-gnu/$n; chmod 755 $D.tmp/lib/x86_64-linux-gnu/$n; done
ls -la $D.tmp/lib64; find $D.tmp/lib64 -maxdepth 2 | head
rm -rf $D; mv $D.tmp $D; du -sh $D; ls -la $D/lib/x86_64-linux-gnu/libc.so.6 $D/lib/x86_64-linux-gnu/ld-linux-x86-64.so.2
ln -sfn ../lib/x86_64-linux-gnu/ld-linux-x86-64.so.2 $D/lib64/ld-linux-x86-64.so.2
mkdir -p $Q/newlibs2 && tar -xf $Q/newlibs2.tar -C $Q/newlibs2
for f in $Q/newlibs2/*; do n=$(basename $f); rm -f $D/lib/x86_64-linux-gnu/$n; cp $f $D/lib/x86_64-linux-gnu/$n; chmod 755 $D/lib/x86_64-linux-gnu/$n; done
