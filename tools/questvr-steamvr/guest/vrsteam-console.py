#!/usr/bin/env python3
import importlib.machinery, importlib.util, json, os, pathlib, sys

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

def main():
    port = devtools_port()
    if not port:
        print("no Steam client running", file=sys.stderr)
        return 2
    os.environ["BL_CDP_PORT"] = port
    expression = "SteamClient.Console.ExecCommand(%s), 'ok'" % json.dumps(" ".join(sys.argv[1:]))
    result = agent().cdp_eval(pathlib.Path("/nonexistent"), expression)
    print(result.get("value"))
    return 0

if __name__ == "__main__":
    sys.exit(main())
