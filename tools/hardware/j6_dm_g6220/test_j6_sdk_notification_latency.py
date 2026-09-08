"""Offline binary check; set DMCAN_ORIGINAL_TEST_LIBRARY on other hosts."""
import ctypes
import hashlib
import os
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest
import j6_sdk_notification_latency as runtime


def test_pinned_private_copy_and_context_selection_without_hardware(tmp_path, monkeypatch):
    source = Path(os.environ.get("DMCAN_ORIGINAL_TEST_LIBRARY", str(
        Path(__file__).resolve().parents[3] / ".codex-tmp/j6_sdk_20260908/libdm_device_original.so")))
    if not source.is_file():
        pytest.skip("DMCAN_ORIGINAL_TEST_LIBRARY must name the pinned vendor binary")
    original = source.read_bytes()
    patched = runtime.patch_binary(original)
    assert hashlib.sha256(patched).hexdigest() == runtime.PATCHED_SHA256
    assert [i for i, (a, b) in enumerate(zip(original, patched)) if a != b] == list(range(0x151E8, 0x151EC))
    output = tmp_path / "private" / "libdm_device.so"
    assert runtime.main(["--source", str(source), "--output", str(output)]) == 0
    assert output.read_bytes() == patched and source.read_bytes() == original
    assert runtime.validate_runtime_library(output, runtime.PATCHED_SHA256) == output.resolve()
    with pytest.raises(ValueError, match="OVERWRITE"):
        runtime.main(["--source", str(source), "--output", str(output)])
    with pytest.raises(ValueError):
        runtime.patch_binary(original[:20] + b"changed" + original[27:])
    with pytest.raises(ValueError):
        runtime.validate_runtime_library(output, runtime.ORIGINAL_SHA256)
    with pytest.raises(ValueError):
        runtime.validate_runtime_library(Path("relative.so"), runtime.PATCHED_SHA256)

    calls = []
    class ContextStructure(ctypes.Structure):
        pass
    class FakeSdkContext:
        def _init_funcs(self):
            calls.append("original_sdk_bindings")
    monkeypatch.setitem(sys.modules, "dmcan.dmcan_def", SimpleNamespace(dmcan_context=ContextStructure))
    def fake_load(path):
        calls.append(path)
        return SimpleNamespace(dmcan_context_create=lambda _: calls.append("fake_context_create"))
    monkeypatch.setattr(runtime.ctypes, "CDLL", fake_load)
    assert runtime.runtime_context_type(FakeSdkContext, {}) is FakeSdkContext
    with pytest.raises(ValueError, match="PAIRED"):
        runtime.runtime_context_type(FakeSdkContext, {runtime.PATH_ENV: str(output)})
    assert not calls
    context_type = runtime.runtime_context_type(FakeSdkContext,
        {runtime.PATH_ENV: str(output), runtime.SHA_ENV: runtime.PATCHED_SHA256})
    assert not calls  # Selection/validation never loads or opens a device.
    context = context_type()
    assert context.dll_path == str(output.resolve())
    assert calls == [str(output.resolve()), "original_sdk_bindings", "fake_context_create"]
    output.write_bytes(patched[:-1] + bytes((patched[-1] ^ 1,)))
    with pytest.raises(ValueError, match="SHA256"):
        context_type()  # Recheck before every open/reconnect; no fallback.
    assert len(calls) == 3
