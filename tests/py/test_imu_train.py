"""Tests for tools/imu_train.py: the tree itself, the session-wise folds, the C export, the CLI.

The cross-check against scikit-learn and the C compile test are skipped when sklearn / a C compiler
is not available; numpy is required for everything here.
"""
import json
import os
import shutil
import subprocess
import sys
import tempfile
import unittest

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
TOOLS = os.path.join(ROOT, "tools")
sys.path.insert(0, TOOLS)

try:
    import numpy as np
    import imu_train as T
except ImportError:
    np = None

try:
    from sklearn.tree import DecisionTreeClassifier
except ImportError:
    DecisionTreeClassifier = None

CC = shutil.which("cc") or shutil.which("gcc")


def run(script, *args, check=True):
    r = subprocess.run([sys.executable, os.path.join(TOOLS, script), *args], capture_output=True, text=True)
    if check:
        assert r.returncode == 0, r.stderr
    return r


def blobs(rng, n_per=150, shift=1.2, n_feat=5):
    """3 overlapping Gaussian classes: a problem where no tree is perfect and ties do not occur."""
    X = np.concatenate([rng.normal(loc=c * shift, scale=1.0, size=(n_per, n_feat)) for c in range(3)])
    y = np.repeat(np.arange(3), n_per)
    return X, y


@unittest.skipIf(np is None, "numpy not installed")
class TreeBasics(unittest.TestCase):
    def test_single_threshold_is_the_midpoint(self):
        X = np.array([[1.0], [2.0], [3.0], [10.0], [11.0], [12.0]])
        y = np.array([0, 0, 0, 1, 1, 1])
        t = T.Tree(max_depth=3, min_leaf=1).fit(X, y, 2)
        self.assertEqual((t.n_nodes(), t.n_leaves(), t.depth()), (3, 2, 1))
        self.assertAlmostEqual(t.threshold[0], 6.5)
        self.assertEqual(list(t.predict(X)), list(y))
        self.assertEqual(list(t.predict(np.array([[6.4], [6.6]]))), [0, 1])

    def test_pure_data_is_one_leaf(self):
        t = T.Tree().fit(np.random.default_rng(0).normal(size=(50, 3)), np.zeros(50, dtype=int), 2)
        self.assertEqual((t.n_nodes(), t.n_leaves()), (1, 1))

    def test_band_needs_two_thresholds_so_depth_two(self):
        rng = np.random.default_rng(1)
        X = rng.uniform(-1, 1, size=(400, 3))
        y = (np.abs(X[:, 0]) > 0.5).astype(int)                 # class 1 outside a band: two cuts on one feature
        d1 = T.Tree(1, 5).fit(X, y, 2)
        d2 = T.Tree(2, 5).fit(X, y, 2)
        self.assertLess((d1.predict(X) == y).mean(), 0.80)
        self.assertEqual((d2.predict(X) == y).mean(), 1.0)

    @unittest.skipIf(DecisionTreeClassifier is None, "scikit-learn not installed")
    def test_xor_behaves_like_scikit_learn_greedy_trees_cannot_split_it_early(self):
        rng = np.random.default_rng(1)
        X = rng.uniform(-1, 1, size=(400, 2))
        y = ((X[:, 0] > 0) ^ (X[:, 1] > 0)).astype(int)
        for depth in (1, 2, 3, 4):
            mine = T.Tree(depth, 5).fit(X, y, 2)
            ref = DecisionTreeClassifier(max_depth=depth, min_samples_leaf=5, random_state=0).fit(X, y)
            self.assertTrue((mine.predict(X) == ref.predict(X)).all(), depth)
        self.assertLess((T.Tree(2, 5).fit(X, y, 2).predict(X) == y).mean(), 0.7)   # greedy CART fails at depth 2

    def test_depth_and_min_leaf_are_respected(self):
        X, y = blobs(np.random.default_rng(2))
        for depth, leaf in ((2, 5), (4, 20), (6, 50)):
            t = T.Tree(depth, leaf).fit(X, y, 3)
            self.assertLessEqual(t.depth(), depth)
            self.assertTrue(all(int(c.sum()) >= leaf for f, c in zip(t.feature, t.counts) if f < 0))

    def test_json_round_trip_predicts_the_same(self):
        X, y = blobs(np.random.default_rng(3))
        t = T.Tree(4, 10).fit(X, y, 3)
        t2 = T.Tree.from_dict(json.loads(json.dumps(t.to_dict([f"f{i}" for i in range(5)], ["a", "b", "c"]))))
        self.assertTrue((t.predict(X) == t2.predict(X)).all())

    @unittest.skipIf(DecisionTreeClassifier is None, "scikit-learn not installed")
    def test_agrees_with_scikit_learn(self):
        rng = np.random.default_rng(4)
        X, y = blobs(rng)
        Xt, yt = blobs(rng)                                   # fresh test set from the same distribution
        for depth, leaf in ((2, 5), (3, 10), (5, 20)):
            mine = T.Tree(depth, leaf).fit(X, y, 3)
            ref = DecisionTreeClassifier(criterion="gini", max_depth=depth, min_samples_leaf=leaf,
                                         random_state=0).fit(X, y)
            for A, B in ((X, ref.predict(X)), (Xt, ref.predict(Xt))):
                agree = (mine.predict(A) == B).mean()
                self.assertGreater(agree, 0.995, f"depth {depth} leaf {leaf}: agreement {agree:.4f}")
            self.assertAlmostEqual((mine.predict(Xt) == yt).mean(), (ref.predict(Xt) == yt).mean(), delta=0.01)


@unittest.skipIf(np is None, "numpy not installed")
class Folds(unittest.TestCase):
    def test_one_session_per_class_is_held_out_and_never_leaks(self):
        y = np.array(["a"] * 30 + ["b"] * 30)
        g = np.array([f"a{i // 10}" for i in range(30)] + [f"b{i // 10}" for i in range(30)])
        folds = T.session_folds(y, g)
        self.assertEqual(len(folds), 3)
        for tr, te in folds:
            self.assertEqual(len(set(g[tr]) & set(g[te])), 0)          # no session on both sides
            self.assertEqual(set(y[te]), {"a", "b"})
            self.assertEqual(len(set(g[te])), 2)
            self.assertEqual(len(tr) + len(te), 60)
        self.assertEqual(sorted(i for _, te in folds for i in te), list(range(60)))   # every window tested once

    def test_fold_count_is_the_smallest_session_count(self):
        y = np.array(["a"] * 20 + ["b"] * 10)
        g = np.array([f"a{i // 10}" for i in range(20)] + ["b0"] * 10)
        self.assertEqual(len(T.session_folds(y, g)), 1)


@unittest.skipIf(np is None or CC is None, "numpy or a C compiler not available")
class CExport(unittest.TestCase):
    def test_exported_c_gives_the_same_classes_as_python(self):
        rng = np.random.default_rng(5)
        X, y = blobs(rng, n_per=200, n_feat=6)
        names = [f"feat{i}" for i in range(6)]
        classes = ["alpha", "beta", "gamma"]
        t = T.Tree(5, 10).fit(X, y, 3)
        with tempfile.TemporaryDirectory() as tmp:
            with open(os.path.join(tmp, "imu_tree.h"), "w") as f:
                f.write(t.to_c(names, classes))
            with open(os.path.join(tmp, "main.c"), "w") as f:
                f.write('#include <stdio.h>\n#include "imu_tree.h"\n'
                        'int main(void){ float f[IMU_N_FEATURES];\n'
                        '  for(;;){ for(int i=0;i<IMU_N_FEATURES;i++) if(scanf("%f",&f[i])!=1) return 0;\n'
                        '    printf("%d\\n", imu_classify(f)); } }\n')
            exe = os.path.join(tmp, "a.out")
            r = subprocess.run([CC, "-std=c99", "-Wall", "-Wextra", "-Werror", "-O1", "-o", exe,
                                os.path.join(tmp, "main.c"), "-I", tmp], capture_output=True, text=True)
            self.assertEqual(r.returncode, 0, r.stderr)
            X32 = X.astype(np.float32)
            # boundary cases: for every split, a row whose feature equals the (float32) threshold exactly
            edge = []
            for node, feat in enumerate(t.feature):
                if feat >= 0:
                    row = X32[node % len(X32)].copy()
                    row[feat] = np.float32(t.threshold[node])
                    edge.append(row)
            self.assertGreater(len(edge), 3)
            X = np.concatenate([X, np.array(edge, dtype=np.float64)])
            X32 = X.astype(np.float32)
            text = "\n".join(" ".join("%.9g" % v for v in row) for row in X32)
            out = subprocess.run([exe], input=text, capture_output=True, text=True).stdout.split()
            got = np.array([int(v) for v in out])
            self.assertEqual(len(got), len(X))
            self.assertTrue((got == t.predict(X, dtype=np.float32)).all())       # exact, float32 arithmetic
            self.assertGreater((got == t.predict(X)).mean(), 0.99)               # and float64 agrees (ties aside)

    def test_header_documents_features_and_classes(self):
        t = T.Tree(2, 1).fit(np.array([[0.0], [1.0]]), np.array([0, 1]), 2)
        h = t.to_c(["speed"], ["slow", "fast"])
        self.assertIn("[0] speed", h)
        self.assertIn("1 = fast", h)
        self.assertIn("IMU_N_CLASSES 2", h)


@unittest.skipIf(np is None, "numpy not installed")
class Cli(unittest.TestCase):
    def make_features(self, tmp, simulated=True):
        dirs = []
        for i, act in enumerate(("still", "walk", "run")):
            for k in range(3):
                seed = 100 * (i + 1) + k
                run("imu_sim.py", act, "--duration", "20", "--seed", str(seed), "--out", tmp)
                d = os.path.join(tmp, f"sim-s{seed:04d}_{act}")
                run("imu_convert.py", d, "--npz")
                dirs.append(d)
        out = os.path.join(tmp, "f.npz")
        run("imu_features.py", *dirs, "--out", out)
        return out

    def test_cli_warns_about_simulated_data_and_writes_model_and_header(self):
        with tempfile.TemporaryDirectory() as tmp:
            feats = self.make_features(tmp)
            model, header = os.path.join(tmp, "m.json"), os.path.join(tmp, "t.h")
            r = run("imu_train.py", feats, "--depth", "3", "--min-leaf", "10", "--model", model, "--export-c", header)
            self.assertIn("SIMULATED DATA", r.stdout)
            self.assertIn("3 folds", r.stdout)
            self.assertIn("confusion matrix", r.stdout)
            with open(model, encoding="utf-8") as f:
                self.assertEqual(json.load(f)["classes"], ["run", "still", "walk"])
            with open(header, encoding="utf-8") as f:
                self.assertIn("imu_classify", f.read())

    def test_unknown_origin_is_mentioned(self):
        with tempfile.TemporaryDirectory() as tmp:
            feats = self.make_features(tmp)
            d = np.load(feats)
            old = os.path.join(tmp, "old.npz")                       # a feature file from before the flag existed
            np.savez(old, **{k: d[k] for k in d.files if k != "simulated"})
            r = run("imu_train.py", old, "--depth", "3", "--min-leaf", "10")
            self.assertNotIn("SIMULATED", r.stdout)
            self.assertIn("does not say whether", r.stdout)

    def test_one_session_per_class_is_refused(self):
        with tempfile.TemporaryDirectory() as tmp:
            run("imu_sim.py", "walk", "--duration", "20", "--out", tmp)
            run("imu_sim.py", "run", "--duration", "20", "--out", tmp)
            dirs = [os.path.join(tmp, "sim-s0001_walk"), os.path.join(tmp, "sim-s0001_run")]
            for d in dirs:
                run("imu_convert.py", d, "--npz")
            out = os.path.join(tmp, "f.npz")
            run("imu_features.py", *dirs, "--out", out)
            r = run("imu_train.py", out, check=False)
            self.assertEqual(r.returncode, 1)
            self.assertIn("at least 2 sessions", r.stderr)


if __name__ == "__main__":
    unittest.main()
