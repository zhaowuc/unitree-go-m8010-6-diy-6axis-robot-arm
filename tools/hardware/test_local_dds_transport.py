"""Current GUI/session entry points must use the verified loopback-only UDP path."""
import os
from pathlib import Path
import re
import shutil
import subprocess
import xml.etree.ElementTree as ET


def test_loopback_udp_profile_and_all_current_entry_points(tmp_path):
    root = Path(__file__).resolve().parents[2]
    templates = root / "tools/hardware/v15_31d_demo_templates"
    canonical = templates / "fastdds_udp_only.xml"
    tree = ET.fromstring(canonical.read_bytes())
    for node in tree.iter():
        node.tag = node.tag.split("}")[-1]
    descriptors = tree.findall("./transport_descriptors/transport_descriptor")
    assert len(descriptors) == 1 and descriptors[0].findtext("type") == "UDPv4"
    assert [n.text for n in descriptors[0].findall("./interfaceWhiteList/address")] == ["127.0.0.1"]
    rtps = tree.find("./participant/rtps")
    assert rtps.findtext("useBuiltinTransports") == "false"
    assert [n.text for n in rtps.findall("./userTransports/transport_id")] == [descriptors[0].findtext("transport_id")]
    entries = [root / "start_arm_gui.sh", root / "tools/start_arm_gui_menu.sh"]
    entries += [p for p in templates.glob("*.sh") if "ROS_LOCALHOST_ONLY=" in p.read_text(encoding="utf-8")]
    assert len(entries) == 9
    for path in entries:
        text = path.read_text(encoding="utf-8")
        assert re.findall(r"(?m)^export[^\n]*\bROS_LOCALHOST_ONLY=(\w+)", text) == ["0"]
        assert "FASTDDS_DEFAULT_PROFILES_FILE=" in text and "FASTRTPS_DEFAULT_PROFILES_FILE=" in text
        assert "RMW_IMPLEMENTATION=rmw_fastrtps_cpp" in text

    # Run only the direct entry's environment block: no ROS, build or devices.
    source = (root / "start_arm_gui.sh").read_text(encoding="utf-8")
    block = source[source.index('LOCAL_DDS_PROFILE='):source.index("export MUJOCO_GL=glfw")]
    bash = shutil.which("bash") or "D:/Git/bin/bash.exe"
    copied = tmp_path / "same profile.xml"
    copied.write_bytes(canonical.read_bytes())
    different = tmp_path / "open-network.xml"
    different.write_text("<profiles/>", encoding="utf-8")
    script = 'set -eu\nfail() { printf "%s\\n" "$*" >&2; exit 2; }\n' + block
    script += '\nprintf "%s\\n%s\\n" "$ROS_LOCALHOST_ONLY" "$FASTDDS_DEFAULT_PROFILES_FILE"\n'
    for external, rmw, success in ((None, None, True), (copied, None, True),
                                    (different, None, False), (None, "rmw_cyclonedds_cpp", False)):
        env = dict(os.environ, REPO_ROOT=root.as_posix())
        for name in ("FASTRTPS_DEFAULT_PROFILES_FILE", "FASTDDS_DEFAULT_PROFILES_FILE", "RMW_IMPLEMENTATION"):
            env.pop(name, None)
        if external:
            env["FASTRTPS_DEFAULT_PROFILES_FILE"] = external.as_posix()
        if rmw:
            env["RMW_IMPLEMENTATION"] = rmw
        result = subprocess.run([bash], input=script, text=True, capture_output=True, env=env)
        assert (result.returncode == 0) is success, result.stderr
        if success:
            assert result.stdout.splitlines() == ["0", canonical.as_posix()]
