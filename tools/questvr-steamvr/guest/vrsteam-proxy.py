#!/usr/bin/env python3
#
import importlib.machinery, importlib.util, json, os, pathlib, re, socket, subprocess, sys, threading, time

LISTEN = ("127.0.0.1", int(sys.argv[1]) if len(sys.argv) > 1 else 27063)
TARGET = ("127.0.0.1", 27062)
ORIGIN = b"Origin: http://127.0.0.1:27062"

def pipe(src, dst):
    try:
        while True:
            data = src.recv(65536)
            if not data:
                break
            dst.sendall(data)
    except OSError:
        pass
    for s in (src, dst):
        try:
            s.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass

def serve(client):
    head = b""
    while b"\r\n\r\n" not in head:
        chunk = client.recv(4096)
        if not chunk:
            client.close()
            return
        head += chunk
    head, rest = head.split(b"\r\n\r\n", 1)
    lines = head.split(b"\r\n")
    request = lines[0].split(b" ")
    if len(request) == 3:
        request[1] = request[1].split(b"?")[0]  # the bogus secret Steam has
        lines[0] = b" ".join(request)
    lines = [l for l in lines if not l.lower().startswith((b"origin:", b"host:"))]
    lines.insert(1, b"Host: 127.0.0.1:27062")
    lines.insert(2, ORIGIN)
    try:
        server = socket.create_connection(TARGET, 5)
        server.settimeout(None)
    except OSError:
        client.close()
        return
    server.sendall(b"\r\n".join(lines) + b"\r\n\r\n" + rest)
    threading.Thread(target=pipe, args=(server, client), daemon=True).start()
    pipe(client, server)

CONNECT_JS = r'''(async () => {
  if (!window.__qdvr) {
    let req;
    window.webpackChunksteamui.push([[Symbol("questdeck")], {}, r => { req = r; }]);
    for (const id of Object.keys(req.m)) {
      const src = req.m[id].toString();
      if (!src.includes('"VRMessages"') || !src.includes("steam_connect")) continue;
      const ex = req(id);
      for (const k in ex) if (ex[k] && typeof ex[k] === "object" && "m_mapCallbacks" in ex[k]) window.__qdvr = ex[k];
    }
    if (!window.__qdvr) return "no VRMessages module";
  }
  const vr = window.__qdvr;
  vr.GetWebSocketURL = async () => "ws://127.0.0.1:%d/";
  if (vr.m_bReady) return "connected";
  vr.m_bClientConnectionFailed = false;
  try { await vr.Connect(); } catch (e) { return "connect failed"; }
  return vr.m_bReady ? "connected now" : "not connected";
})()''' % LISTEN[1]

def agent():
    loader = importlib.machinery.SourceFileLoader("droiddeck_agent", "/usr/local/bin/droiddeck-agent")
    spec = importlib.util.spec_from_loader(loader.name, loader)
    module = importlib.util.module_from_spec(spec)
    loader.exec_module(module)
    return module

def devtools_port():
    for pid in os.listdir("/proc"):
        try:
            argv = open("/proc/%s/cmdline" % pid, "rb").read().split(b"\0")
        except OSError:
            continue
        if argv and argv[0].endswith(b"steam") and b"-devtools-port" in argv:
            return argv[argv.index(b"-devtools-port") + 1].decode()
    return None

Q = "/root/.local/share/questdeck-vr"
MANIFEST = Q + "/config/steamapps.vrmanifest"
APPCONFIG = Q + "/config/appconfig.json"
LAUNCH_REQUEST = Q + "/launch.request"
ART = "/root/.local/share/Steam/appcache/librarycache/%d"
VR_APPS_JS = r'''(() => {
  const installed = appStore.allApps.filter(a => (a.per_client_data || []).some(c => c.installed));
  if (!installed.length) return "null";
  return JSON.stringify(installed.filter(a => a.vr_supported)
    .map(a => ({appid: a.appid, name: a.display_name, played: a.rt_last_time_played || 0})));
})()'''
LAUNCH_JS = r'''(async (appid) => {
  const options = await SteamClient.Apps.GetLaunchOptionsForApp(appid).catch(() => []);
  const pick = options.find(o => o.eType == 5) || options.find(o => o.bIsVRLaunchOption);
  SteamClient.Apps.RunGame(String(appid), "", pick ? pick.nIndex : -1, 100);
  return pick ? pick.nIndex : -1;
})(%s)'''
VRCMD = Q + "/bin/vrcmd.sh"

def write_if_changed(path, text):
    try:
        if open(path).read() == text:
            return False
    except OSError:
        pass
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path + ".tmp", "w") as f:
        f.write(text)
    os.replace(path + ".tmp", path)
    return True

def art(appid):
    folder = ART % appid
    for name in ("library_600x900.jpg", "header.jpg", "library_hero.jpg"):
        if os.path.isfile(os.path.join(folder, name)):
            return os.path.join(folder, name)
    return None

def write_manifest(apps):
    entries = []
    for app in apps:
        entry = {
            "app_key": "steam.app.%d" % app["appid"],
            "launch_type": "binary",
            "binary_path_linux": Q + "/bin/vrlaunch.sh",
            "arguments": str(app["appid"]),
            "strings": {"en_us": {"name": app["name"]}},
            "last_played_time": str(app["played"]),  # SteamVR reads it as a string
        }
        image = art(app["appid"])
        if image:
            entry["image_path"] = image
        entries.append(entry)
    changed = write_if_changed(MANIFEST, json.dumps({"source": "steam", "applications": entries}, indent=2) + "\n")
    write_if_changed(APPCONFIG, json.dumps({"manifest_paths": [MANIFEST]}, indent=2) + "\n")
    return changed

def register_manifest():
    """A running vrserver reads the new list now; one starting later reads it from appconfig.json."""
    if subprocess.run(["pgrep", "-f", "[v]rserver --keepalive"], capture_output=True).returncode != 0:
        return
    subprocess.Popen(["sh", VRCMD, "--appmanifest", MANIFEST], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

def launch_requested():
    try:
        appid = open(LAUNCH_REQUEST).read().strip()
        os.unlink(LAUNCH_REQUEST)
    except OSError:
        return None
    return appid if appid.isdigit() else None

def keep_steam_connected():
    module, last, listed_at = agent(), None, 0
    where = pathlib.Path("/nonexistent")
    while True:
        try:
            port = devtools_port()
            if port:
                os.environ["BL_CDP_PORT"] = port
                state = module.cdp_eval(where, CONNECT_JS)["value"]
                if time.monotonic() - listed_at > 30:
                    apps = json.loads(module.cdp_eval(where, VR_APPS_JS)["value"])
                    if apps is not None:
                        if write_manifest(apps):
                            print("[vrsteam] VR library: %s" % ", ".join(a["name"] for a in apps), flush=True)
                            register_manifest()
                        listed_at = time.monotonic()
                appid = launch_requested()
                if appid:
                    option = module.cdp_eval(where, LAUNCH_JS % appid)["value"]
                    print("[vrsteam] launching %s (launch option %s)" % (appid, option), flush=True)
            else:
                state = "no Steam client"
        except Exception as e:  # Steam restarting, DevTools busy: tried again shortly
            state = "error: %s" % e
        if state != last:
            print("[vrsteam] " + str(state), flush=True)
            last = state
        time.sleep(2)

def main():
    threading.Thread(target=keep_steam_connected, daemon=True).start()
    listener = socket.socket()
    listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    listener.bind(LISTEN)
    listener.listen(8)
    while True:
        client, _ = listener.accept()
        threading.Thread(target=serve, args=(client,), daemon=True).start()

if __name__ == "__main__":
    main()
