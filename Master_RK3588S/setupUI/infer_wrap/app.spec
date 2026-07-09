# -*- mode: python ; coding: utf-8 -*-
import sys
from PyInstaller.utils.hooks import collect_all
import os

sys.setrecursionlimit(sys.getrecursionlimit() * 5)
datas = []
binaries = []
hiddenimports = []

tmp_ret = collect_all('glfw')
datas += tmp_ret[0]; binaries += tmp_ret[1]; hiddenimports += tmp_ret[2]

tmp_ret_scipy = collect_all('scipy')
datas += tmp_ret_scipy[0]; binaries += tmp_ret_scipy[1]; hiddenimports += tmp_ret_scipy[2]

tmp_ret_trimesh = collect_all('trimesh')
datas += tmp_ret_trimesh[0]; binaries += tmp_ret_trimesh[1]; hiddenimports += tmp_ret_trimesh[2]

a = Analysis(
    ['app.py'],
    pathex=[],
    binaries=binaries,
    datas=datas,
    hiddenimports=hiddenimports + [
        'hwid', 
        'license_manager',
        'scipy._lib.array_api_compat.numpy.fft',
        'scipy._lib.array_api_compat.numpy.linalg',
        'trimesh',
        # ---------- 标准库模块 ----------
        # scipy.sparse 和 trimesh/exceptions.py 在运行时会动态 import 这些模块，
        # 不能在 excludes 里排除任何标准库模块！
        'unittest', 'unittest.mock', 'unittest.case', 'unittest.suite',
        'unittest.runner', 'unittest.loader', 'unittest.result',
        'pydoc', 'doctest', 'inspect', 'dis', 'opcode',
    ],
    hookspath=['hooks'],
    hooksconfig={},
    runtime_hooks=[],
    excludes=[
        # ---- matplotlib: not used by app.py; system version conflicts with venv numpy >=2.0 ----
        'matplotlib', 'matplotlib.pyplot', 'matplotlib.figure', 'matplotlib.axes',
        'matplotlib.backends', 'matplotlib.ticker', 'matplotlib.scale',
        'matplotlib.transforms', 'matplotlib._path', 'matplotlib.cbook',
        # ---- other heavy / unused third-party packages ----
        'tkinter', '_tkinter', 'Tkinter',
        'wx', 'PyQt5', 'PyQt6', 'PySide2', 'PySide6',
        'IPython', 'notebook', 'jupyter',
        'pandas', 'sklearn', 'sklearn.utils',
        # 重要: 不要排除任何 Python 标准库 (pydoc/doctest/unittest 等)。
        # scipy.sparse 和 trimesh 在运行时会动态 import 它们，排除后会出现 ModuleNotFoundError。
    ],
    noarchive=False,
    optimize=0,
)
pyz = PYZ(a.pure)

# ----------------- ARM/ROCKCHIP FIX -----------------
# Remove standard system and graphics libraries so PyInstaller won't 
# shadow the native Rockchip GPU/DRI drivers on the OrangePi host.
def filter_binaries(binaries):
    excluded_prefixes = [
        'libGL.', 'libEGL', 'libGLES', 'libglapi', 'libdrm', 'libgbm', 
        'libX11', 'libxcb', 'libstdc++', 'libgcc_s', 'libgomp', 'libvulkan'
    ]
    cleaned = []
    for b in binaries:
        dest_name = os.path.basename(b[0])
        if any(dest_name.startswith(p) for p in excluded_prefixes):
            print(f"[EXCLUDED] {dest_name}")
        else:
            cleaned.append(b)
    return cleaned

a.binaries = filter_binaries(a.binaries)
# ----------------------------------------------------

exe = EXE(
    pyz,
    a.scripts,
    a.binaries,
    a.datas,
    [],
    name='app',
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=True,
    upx_exclude=[],
    runtime_tmpdir=None,
    console=True,
    disable_windowed_traceback=False,
    argv_emulation=False,
    target_arch=None,
    codesign_identity=None,
    entitlements_file=None,
)
