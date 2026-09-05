import copy
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
from v15_31c_correct_j4_direction import corrected_documents


def test_j4_migration_preserves_raw_zero_and_other_joints():
    z = {"mapping": {"signs": {"J4": 1, "J3": 1}},
         "motors": {"J4": {"raw_position_rad": -2.2}}}
    h = {"motors": {"J4": {"logical_position_rad": 0.1},
                    "J3": {"logical_position_rad": 0.2}}}
    i = {"关节位置_弧度": {"J4": 0.3, "J3": 0.4}}
    original = copy.deepcopy((z, h, i))
    zz, hh, ii, sha = corrected_documents(z, h, i, "2026-09-05T00:00:00Z")
    assert (z, h, i) == original
    assert zz["motors"] == z["motors"]
    assert zz["mapping"]["signs"] == {"J4": -1, "J3": 1}
    assert hh["motors"]["J4"]["logical_position_rad"] == -0.1
    assert hh["motors"]["J3"] == h["motors"]["J3"]
    assert ii["关节位置_弧度"] == {"J4": -0.3, "J3": 0.4}
    assert ii["会话标识"] == "persistent:" + sha[:16]
