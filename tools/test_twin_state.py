"""Run firmware reference/status policy on Windows with installed LLVM, no hardware."""
import ctypes
from pathlib import Path
import subprocess
import tempfile

source = Path(__file__).with_suffix('.c')
clang = Path(r'C:\Program Files\LLVM\bin\clang.exe')
with tempfile.TemporaryDirectory(prefix='twin-state-') as folder:
    library = Path(folder) / 'twin-state.dll'
    subprocess.run([str(clang), '-shared', '-nostdlib', '-fuse-ld=lld',
                    '-Wl,/noentry', '-Wall', '-Wextra', '-Werror',
                    str(source), '-o', str(library)], check=True)
    dll = ctypes.CDLL(str(library))
    result = dll.twin_state_contract_test()
    # Release the DLL before TemporaryDirectory removes the test output.
    handle = dll._handle
    del dll
    ctypes.windll.kernel32.FreeLibrary.argtypes = [ctypes.c_void_p]
    ctypes.windll.kernel32.FreeLibrary(ctypes.c_void_p(handle))
    if result:
        raise AssertionError(f'Firmware policy assertion failed at C line {result}')
print('PASS firmware reference validity, no automatic restore, fixed origin and compact HMI contract')
