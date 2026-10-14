import os
import runpy
import subprocess
import sys

KEY = "STEAM_COMPAT_DATA_PATH"
os.environ[KEY] = os.environ.pop("QF_" + KEY)

def without_key(env):
    env = dict(os.environ if env is None else env)
    env.pop(KEY, None)
    return env

_popen_init = subprocess.Popen.__init__

def popen_init(self, *args, **kwargs):
    kwargs["env"] = without_key(kwargs.get("env"))
    _popen_init(self, *args, **kwargs)

subprocess.Popen.__init__ = popen_init
_execve = os.execve
os.execve = lambda path, args, env: _execve(path, args, without_key(env))
os.execv = lambda path, args: _execve(path, args, without_key(None))
os.execvp = lambda file, args: os.execvpe(file, args, without_key(None))

script = sys.argv[1]
sys.argv = sys.argv[1:]
sys.path.insert(0, os.path.dirname(os.path.abspath(script)))  # Proton's own modules (filelock)
runpy.run_path(script, run_name="__main__")
