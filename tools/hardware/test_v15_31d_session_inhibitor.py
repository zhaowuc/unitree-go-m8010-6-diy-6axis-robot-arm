"""The inhibitor precedes execution and survives cleanup; no D-Bus or hardware."""
from io import StringIO
from types import SimpleNamespace

import pytest
import v15_31d_demo_session as launcher


def test_session_inhibitor_acquires_before_work_and_releases_after_cleanup(monkeypatch):
    for failure in (None, "stage failure", "inhibitor denied"):
        order = []
        class Input(StringIO):
            def close(self):
                order.append("stdin-eof")
                super().close()
        def popen(command, **kwargs):
            assert command[:3] == ["/usr/bin/python3", "-u", "-c"]
            assert "'Inhibit'" in command[3] and "'Uninhibit'" in command[3]
            assert kwargs["stdin"] == launcher.subprocess.PIPE and kwargs["close_fds"]
            order.append("acquire")
            return SimpleNamespace(stdin=Input(),
                stdout=StringIO("" if failure == "inhibitor denied" else "READY\n"),
                stderr=StringIO("Access denied" if failure == "inhibitor denied" else ""),
                poll=lambda: 1 if failure == "inhibitor denied" else None,
                wait=lambda **_: order.append("released"))
        monkeypatch.setattr(launcher.subprocess, "Popen", popen)
        monkeypatch.setattr(launcher.select, "select", lambda readers, *_: (readers, [], []))
        def execute():
            with launcher.session_auto_sleep_inhibited():
                try:
                    order.append("work")
                    if failure == "stage failure":
                        raise RuntimeError(failure)
                finally:
                    order.append("hardware-cleanup")
        if failure:
            with pytest.raises(RuntimeError, match="Access denied" if failure == "inhibitor denied" else failure):
                execute()
        else:
            execute()
        assert order == (["acquire"] if failure == "inhibitor denied" else ["acquire", "work", "hardware-cleanup"]) + ["stdin-eof", "released"]
