"""Reproduce a private, hash-pinned SDK idle-notification delay correction.

The vendor v1.1.0 x86_64 binary sleeps 100 ms when its notification queues are
empty. Only that timespec is changed to 1 ms; no code, USB timeout, CAN packet,
motor setting or installed library is rewritten. No SDK/device calls at import.
"""
import argparse
import ctypes
import hashlib
import json
import os
from pathlib import Path
import struct


ORIGINAL_SHA256 = "bec4ca4421f5366c90fa11cc34f2f683ed71a6372e15c265e7e4b17e720607dd"
PATCHED_SHA256 = "6723c21c2eec34f5a17baebe4a3dde68f518782d941c416a025e41247838f755"
LIBRARY_SIZE = 132032
TIMESPEC_OFFSET = 0x151E0
REFERENCE_OFFSET = 0x10318
REFERENCE_BYTES = bytes.fromhex("660f6f05c04e0000")
NANOSLEEP_CALL_OFFSET = 0x10330
NANOSLEEP_CALL_BYTES = bytes.fromhex("e84b77ffff")
PATH_ENV = "DMCAN_RUNTIME_LIBRARY_PATH"
SHA_ENV = "DMCAN_RUNTIME_LIBRARY_SHA256"


def validate_binary(data, *, patched):
    expected = PATCHED_SHA256 if patched else ORIGINAL_SHA256
    if len(data) != LIBRARY_SIZE or hashlib.sha256(data).hexdigest() != expected:
        raise ValueError("DMCAN_RUNTIME_LENGTH_OR_SHA256_MISMATCH")
    if data[:6] != b"\x7fELF\x02\x01" or struct.unpack_from("<HH", data, 16) != (3, 62):
        raise ValueError("DMCAN_RUNTIME_REQUIRES_ELF64_LE_X86_64_SHARED_OBJECT")
    shoff = struct.unpack_from("<Q", data, 40)[0]
    size, count, strings_index = struct.unpack_from("<HHH", data, 58)
    if size != 64 or not strings_index < count or shoff + size*count > len(data):
        raise ValueError("DMCAN_RUNTIME_SECTION_TABLE_INVALID")
    sections = [struct.unpack_from("<IIQQQQIIQQ", data, shoff + i*size) for i in range(count)]
    strings = sections[strings_index]
    names = data[strings[4]:strings[4]+strings[5]]
    sections = {names[s[0]:].split(b"\0", 1)[0]: s for s in sections}
    text, rodata = sections[b".text"], sections[b".rodata"]
    if text[2:6] != (6, 0x82C0, 0x82C0, 0xCBE8) or rodata[2:6] != (2, 0x15000, 0x15000, 0x815):
        raise ValueError("DMCAN_RUNTIME_SECTION_LAYOUT_CHANGED")
    instruction = data[REFERENCE_OFFSET:REFERENCE_OFFSET+8]
    displacement = struct.unpack_from("<i", instruction, 4)[0]
    if (instruction != REFERENCE_BYTES
            or data[text[4]:text[4]+text[5]].count(REFERENCE_BYTES) != 1
            or REFERENCE_OFFSET + 8 + displacement != TIMESPEC_OFFSET
            or data[NANOSLEEP_CALL_OFFSET:NANOSLEEP_CALL_OFFSET+5] != NANOSLEEP_CALL_BYTES):
        raise ValueError("DMCAN_RUNTIME_NOTIFICATION_REFERENCE_CHANGED")
    value = struct.pack("<qq", 0, 1_000_000 if patched else 100_000_000)
    if data[TIMESPEC_OFFSET:TIMESPEC_OFFSET+16] != value or (not patched and data.count(value) != 1):
        raise ValueError("DMCAN_RUNTIME_NOTIFICATION_TIMESPEC_CHANGED")


def patch_binary(data):
    validate_binary(data, patched=False)
    result = bytearray(data)
    result[TIMESPEC_OFFSET+8:TIMESPEC_OFFSET+16] = struct.pack("<q", 1_000_000)
    result = bytes(result)
    validate_binary(result, patched=True)
    if [i for i, (a, b) in enumerate(zip(data, result)) if a != b] != list(range(0x151E8, 0x151EC)):
        raise ValueError("DMCAN_RUNTIME_UNEXPECTED_PATCH_DIFFERENCE")
    return result


def validate_runtime_library(path, expected_sha256):
    """Validate the explicit private copy without loading it or opening USB."""
    candidate = Path(path)
    if not candidate.is_absolute() or candidate.is_symlink() or not candidate.is_file():
        raise ValueError("DMCAN_RUNTIME_REQUIRES_ABSOLUTE_REGULAR_PRIVATE_FILE")
    if expected_sha256 != PATCHED_SHA256 or candidate.stat().st_size != LIBRARY_SIZE:
        raise ValueError("DMCAN_RUNTIME_PRIVATE_SHA256_OR_LENGTH_INVALID")
    validate_binary(candidate.read_bytes(), patched=True)
    return candidate.resolve(strict=True)


def runtime_context_type(default_context, environ=None):
    """Use the same SDK C bindings with a selected library, before USB opens."""
    env = os.environ if environ is None else environ
    path, sha = env.get(PATH_ENV), env.get(SHA_ENV)
    if path is None and sha is None:
        return default_context
    if not path or not sha:
        raise ValueError("DMCAN_RUNTIME_PATH_AND_SHA256_MUST_BE_PAIRED")
    selected = validate_runtime_library(path, sha)

    class PrivateRuntimeContext(default_context):
        def __init__(self):
            # Match dmcan-sdk 1.0.4 context initialization, selecting only the
            # CDLL path. No monkeypatch, LD_PRELOAD, or in-memory binary edit.
            from dmcan.dmcan_def import dmcan_context
            self.dll_path = str(validate_runtime_library(selected, sha))
            self.dll = ctypes.CDLL(self.dll_path)
            self._ctx = ctypes.POINTER(dmcan_context)()
            self._devices = []
            self._init_funcs()
            print(f"DMCAN_PRIVATE_RUNTIME path={self.dll_path} sha256={sha} notification_idle_ms=1", flush=True)
            self.dll.dmcan_context_create(self._ctx)

    return PrivateRuntimeContext


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    if args.source.is_symlink() or not args.source.is_file() or args.source.stat().st_size != LIBRARY_SIZE:
        raise ValueError("DMCAN_RUNTIME_SOURCE_FILE_INVALID")
    output = args.output.resolve()
    if output == args.source.resolve() or output.exists() or args.output.is_symlink():
        raise ValueError("DMCAN_RUNTIME_REFUSE_OVERWRITE")
    patched = patch_binary(args.source.read_bytes())
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("xb") as stream:
        stream.write(patched)
    validate_runtime_library(output, PATCHED_SHA256)
    print(json.dumps({"status": "PASS", "source": str(args.source.resolve()), "source_sha256": ORIGINAL_SHA256,
        "path": str(output), "sha256": PATCHED_SHA256, "changed_file_offsets": [hex(i) for i in range(0x151E8, 0x151EC)],
        "notification_idle_ms": {"before": 100, "after": 1}, "hardware_accessed": False}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
