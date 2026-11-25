"""tree_id / IMU_TREE_SIM_DATA in the exported header (sent in every result frame)."""
import json
import os
import shutil
import subprocess
import sys
import tempfile
import unittest

import numpy as np

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "tools"))
import imu_train as T  # noqa: E402

FEATURES = ["f%d" % i for i in range(6)]
CLASSES = ["a", "b", "c"]


def small_tree(seed=0, depth=3):
    rng = np.random.RandomState(seed)
    X = rng.randn(600, len(FEATURES))
    y = ((X[:, 0] > 0).astype(int) + (X[:, 3] > 0.3).astype(int))
    return T.Tree(depth, 10).fit(X, y, 3)


class TreeIdTest(unittest.TestCase):
    def test_header_model_and_json_agree(self):
        t = small_tree()
        hdr = t.to_c(FEATURES, CLASSES, simulated=True)
        tid = t.tree_id(FEATURES)
        self.assertIn("#define IMU_TREE_ID 0x%04Xu" % tid, hdr)
        self.assertIn("#define IMU_TREE_SIM_DATA 1", hdr)
        self.assertEqual(T.tree_id_from_header(hdr), tid)
        t2 = T.Tree.from_dict(json.loads(json.dumps(t.to_dict(FEATURES, CLASSES))))
        self.assertEqual(t2.tree_id(FEATURES), tid)

    def test_id_does_not_depend_on_the_simulated_warning(self):
        t = small_tree()
        self.assertEqual(T.tree_id_from_header(t.to_c(FEATURES, CLASSES, simulated=True)),
                         T.tree_id_from_header(t.to_c(FEATURES, CLASSES, simulated=False)))
        self.assertIn("#define IMU_TREE_SIM_DATA 0", t.to_c(FEATURES, CLASSES, simulated=False))

    def test_different_trees_get_different_ids(self):
        ids = {small_tree(seed).tree_id(FEATURES) for seed in range(8)}
        self.assertGreater(len(ids), 6)

    def test_committed_header_has_matching_id(self):
        path = os.path.join(ROOT, "App", "classify", "imu_tree.h")
        if not os.path.exists(path):
            self.skipTest("no App/classify/imu_tree.h")
        with open(path, encoding="utf-8") as f:
            text = f.read()
        self.assertIn("#define IMU_TREE_ID 0x%04Xu" % T.tree_id_from_header(text), text)
        sim = "TRAINED ON SIMULATED DATA" in text
        self.assertIn("#define IMU_TREE_SIM_DATA %d" % (1 if sim else 0), text)

    @unittest.skipUnless(shutil.which("gcc"), "gcc not found")
    def test_header_compiles_and_exposes_the_id(self):
        t = small_tree()
        with tempfile.TemporaryDirectory() as tmp:
            with open(os.path.join(tmp, "imu_tree.h"), "w") as f:
                f.write(t.to_c(FEATURES, CLASSES, simulated=True))
            with open(os.path.join(tmp, "m.c"), "w") as f:
                f.write('#include <stdio.h>\n#include "imu_tree.h"\n'
                        'int main(void){printf("%04x %d\\n",(unsigned)IMU_TREE_ID,IMU_TREE_SIM_DATA);return 0;}\n')
            subprocess.run(["gcc", "-std=c99", "-Wall", "-Wextra", "-Werror", "-o", os.path.join(tmp, "m"),
                            os.path.join(tmp, "m.c")], check=True)
            out = subprocess.run([os.path.join(tmp, "m")], capture_output=True, text=True, check=True).stdout
        self.assertEqual(out.split(), ["%04x" % t.tree_id(FEATURES), "1"])


if __name__ == "__main__":
    unittest.main()
