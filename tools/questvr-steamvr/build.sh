#!/bin/sh
set -e
here=$(cd "$(dirname "$0")" && pwd)
out=${OUT:-$here/out/questdeck}
mkdir -p "$out/bin/linux64" "$out/resources/settings"
python -m ziglang c++ -target x86_64-linux-gnu.2.17 -std=c++17 -shared -fPIC -O2 -s -fvisibility=hidden -Wall -Wno-nullability-completeness -Wno-unused-private-field \
    -I"$OPENVR_INCLUDE" -I"$VULKAN_INCLUDE" -I"$here/../../app/src/main/cpp/questvr" \
    -o "$out/bin/linux64/driver_questdeck.so" "$here/driver/driver_questdeck.cpp" -ldl -lpthread
mkdir -p "$out/layer"
python -m ziglang cc -target aarch64-linux-gnu.2.17 -shared -fPIC -O2 -s -fvisibility=hidden -Wall -Wno-nullability-completeness     -I"$VULKAN_INCLUDE" -o "$out/layer/libVkLayer_questdeck_steamvr.so" "$here/layer/questdeck_steamvr_layer.c"
cp "$here/layer/VkLayer_questdeck_steamvr.json" "$out/layer/"
python -m ziglang cc -target aarch64-linux-gnu.2.17 -shared -fPIC -O2 -s -Wall -Wno-nullability-completeness     -o "$out/layer/libquestdeck_kgslts.so" "$here/kgslts/questdeck_kgslts.c" -ldl -lpthread
python -m ziglang cc -target x86_64-linux-gnu.2.17 -shared -fPIC -O2 -s -Wall -Wno-nullability-completeness     -o "$out/layer/libquestdeck_vrenv.so" "$here/shim/questdeck_vrenv.c" "$here/shim/questdeck_sysvsem.c" -ldl
python -m ziglang cc -target x86_64-linux-gnu.2.17 -shared -fPIC -O2 -s -Wall -Wno-nullability-completeness -DQUESTDECK_GAME -o "$out/layer/libquestdeck_vrgame.so" "$here/shim/questdeck_vrenv.c" -ldl -lpthread
cp "$here/driver/driver.vrdrivermanifest" "$out/"
cp "$here/driver/resources/settings/default.vrsettings" "$out/resources/settings/"
echo "built $out"
