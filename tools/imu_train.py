#!/usr/bin/env python3
"""Train and evaluate a small decision tree on window features (needs numpy; no scikit-learn).

    python3 tools/imu_train.py data/features_sim.npz --depth 4
    python3 tools/imu_train.py data/features_sim.npz --depth 4 --model data/tree.json --export-c data/imu_tree.h

Evaluation is by SESSION: in fold k the k-th session of every class is held out, the tree is trained on
all the other sessions.  Windows of one session overlap and look alike, so a random split of windows
would put near-copies of the test windows into the training set and report an accuracy that does not
mean anything.  --export-c writes the trained tree as plain C (nested if / else on float features).

If the features come from simulated sessions (tools/imu_sim.py) the accuracy only describes the
simulator, not the real world; the tool says so first thing.
"""
import argparse
import json
import sys

import numpy as np


class Tree:
    """CART classifier: Gini impurity, binary splits `x[feature] <= threshold`, midpoints as thresholds."""

    def __init__(self, max_depth: int = 4, min_leaf: int = 20):
        self.max_depth, self.min_leaf = max_depth, min_leaf
        self.feature, self.threshold, self.left, self.right, self.counts = [], [], [], [], []
        self.gain = None
        self.n_classes = 0

    # ---- training -------------------------------------------------------------------------------
    def fit(self, X: np.ndarray, y: np.ndarray, n_classes: int):
        self.n_classes = n_classes
        self.feature, self.threshold, self.left, self.right, self.counts = [], [], [], [], []
        self.gain = np.zeros(X.shape[1])
        self._grow(X, y, np.arange(len(y)), 0)
        return self

    def _new(self, counts):
        self.feature.append(-1)
        self.threshold.append(0.0)
        self.left.append(-1)
        self.right.append(-1)
        self.counts.append(counts)
        return len(self.feature) - 1

    def _best_split(self, X, y, idx):
        n, K = len(idx), self.n_classes
        eye = np.eye(K)
        parent = eye[y[idx]].sum(axis=0)
        parent_gini = 1.0 - float(((parent / n) ** 2).sum())
        best = (np.inf, -1, 0.0)
        nl = np.arange(1, n)
        nr = n - nl
        for j in range(X.shape[1]):
            x = X[idx, j]
            order = np.argsort(x, kind="stable")
            xs = x[order]
            cl = np.cumsum(eye[y[idx][order]], axis=0)[:-1]
            cr = parent - cl
            score = (nl * (1.0 - (cl ** 2).sum(axis=1) / nl ** 2) +
                     nr * (1.0 - (cr ** 2).sum(axis=1) / nr ** 2)) / n
            valid = (xs[:-1] < xs[1:]) & (nl >= self.min_leaf) & (nr >= self.min_leaf)
            if not valid.any():
                continue
            score = np.where(valid, score, np.inf)
            i = int(np.argmin(score))
            if score[i] < best[0]:
                best = (float(score[i]), j, (float(xs[i]) + float(xs[i + 1])) / 2.0)
        return parent_gini, best

    def _grow(self, X, y, idx, depth):
        counts = np.bincount(y[idx], minlength=self.n_classes)
        node = self._new(counts)
        if depth >= self.max_depth or counts.max() == len(idx) or len(idx) < 2 * self.min_leaf:
            return node
        parent_gini, (score, j, thr) = self._best_split(X, y, idx)
        if j < 0 or not score < parent_gini - 1e-12:
            return node
        self.gain[j] += len(idx) * (parent_gini - score)
        mask = X[idx, j] <= thr
        self.feature[node], self.threshold[node] = j, thr
        self.left[node] = self._grow(X, y, idx[mask], depth + 1)
        self.right[node] = self._grow(X, y, idx[~mask], depth + 1)
        return node

    # ---- prediction -----------------------------------------------------------------------------
    def predict(self, X: np.ndarray, dtype=np.float64) -> np.ndarray:
        """dtype=np.float32 reproduces the arithmetic of the exported C code."""
        X = np.asarray(X, dtype=dtype)
        feat = np.array(self.feature)
        thr = np.array(self.threshold, dtype=dtype)
        left, right = np.array(self.left), np.array(self.right)
        leaf_class = np.array([int(np.argmax(c)) for c in self.counts])
        node = np.zeros(len(X), dtype=np.int64)
        rows = np.arange(len(X))
        while True:
            internal = feat[node] >= 0
            if not internal.any():
                break
            go_left = X[rows, np.maximum(feat[node], 0)] <= thr[node]
            nxt = np.where(go_left, left[node], right[node])
            node = np.where(internal, nxt, node)
        return leaf_class[node]

    # ---- introspection / export -----------------------------------------------------------------
    def n_nodes(self):
        return len(self.feature)

    def n_leaves(self):
        return sum(1 for f in self.feature if f < 0)

    def depth(self, node=0):
        if self.feature[node] < 0:
            return 0
        return 1 + max(self.depth(self.left[node]), self.depth(self.right[node]))

    def to_dict(self, features, classes):
        return {"features": list(features), "classes": list(classes), "max_depth": self.max_depth,
                "min_leaf": self.min_leaf, "feature": self.feature, "threshold": self.threshold,
                "left": self.left, "right": self.right, "counts": [list(map(int, c)) for c in self.counts]}

    @classmethod
    def from_dict(cls, d):
        t = cls(d["max_depth"], d["min_leaf"])
        t.feature, t.threshold, t.left, t.right = d["feature"], d["threshold"], d["left"], d["right"]
        t.counts = [np.array(c) for c in d["counts"]]
        t.n_classes = len(d["classes"])
        return t

    def to_c(self, features, classes, name="imu_classify") -> str:
        """Plain C: float features in, class index out.  Thresholds are float32 literals."""
        out = ["/* Generated by tools/imu_train.py - do not edit.",
               " * Input: f[%d] features in this order:" % len(features)]
        out += [" *   [%d] %s" % (i, n) for i, n in enumerate(features)]
        out += [" * Output: class index:"]
        out += [" *   %d = %s" % (i, c) for i, c in enumerate(classes)]
        out += [" */", "#ifndef IMU_TREE_H", "#define IMU_TREE_H", "",
                "#define IMU_N_FEATURES %d" % len(features), "#define IMU_N_CLASSES %d" % len(classes), ""]
        out += ["static const char *const imu_class_name[IMU_N_CLASSES] = {"] + \
               ['    "%s",' % c for c in classes] + ["};", ""]
        out += ["static inline int %s(const float f[IMU_N_FEATURES])" % name, "{"]

        def emit(node, ind):
            pad = "    " * ind
            if self.feature[node] < 0:
                out.append("%sreturn %d;" % (pad, int(np.argmax(self.counts[node]))))
                return
            thr = np.float32(self.threshold[node])
            out.append("%sif (f[%d] <= %sf) {  /* %s */" % (pad, self.feature[node], repr(float(thr)),
                                                           features[self.feature[node]]))
            emit(self.left[node], ind + 1)
            out.append("%s} else {" % pad)
            emit(self.right[node], ind + 1)
            out.append("%s}" % pad)

        emit(0, 1)
        out += ["}", "", "#endif /* IMU_TREE_H */", ""]
        return "\n".join(out)


def session_folds(y: np.ndarray, groups: np.ndarray):
    """Fold k holds out the k-th session (sorted by name) of every class. -> list of (train_idx, test_idx)."""
    per_class = {c: sorted(set(groups[y == c])) for c in sorted(set(y))}
    n_folds = min(len(v) for v in per_class.values())
    folds = []
    for k in range(n_folds):
        test_groups = {v[k] for v in per_class.values()}
        test = np.isin(groups, list(test_groups))
        folds.append((np.where(~test)[0], np.where(test)[0]))
    return folds


def confusion(y_true: np.ndarray, y_pred: np.ndarray, n: int) -> np.ndarray:
    m = np.zeros((n, n), dtype=np.int64)
    np.add.at(m, (y_true, y_pred), 1)
    return m


def print_confusion(m: np.ndarray, classes):
    w = max(6, max(len(c) for c in classes) + 1)
    print("  true \\ pred " + "".join(f"{c:>{w}s}" for c in classes) + "   recall")
    for i, c in enumerate(classes):
        tot = m[i].sum()
        print(f"  {c:11s} " + "".join(f"{v:>{w}d}" for v in m[i]) + f"   {m[i, i] / tot:.3f}" if tot else f"  {c}: none")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("features", help="npz made by tools/imu_features.py")
    ap.add_argument("--depth", type=int, default=4)
    ap.add_argument("--min-leaf", type=int, default=20)
    ap.add_argument("--model", default=None, help="write the tree trained on ALL sessions as JSON")
    ap.add_argument("--export-c", default=None, metavar="PATH", help="write that tree as a C header")
    args = ap.parse_args(argv)

    d = np.load(args.features)
    X, y_str, groups = d["X"], d["y"], d["group"]
    names = [str(n) for n in d["features"]]
    classes = sorted(set(y_str))
    y = np.array([classes.index(v) for v in y_str])
    sim = d["simulated"] if "simulated" in d.files else None
    if sim is not None and sim.any():
        print("!" * 78)
        print("! SIMULATED DATA" + (" (partly)" if not sim.all() else "") +
              ": the accuracy below describes the simulator, not the real world.")
        print("!" * 78)
    elif sim is None:
        print("note: this feature file does not say whether the data is real or simulated")

    folds = session_folds(y_str, groups)
    if len(folds) < 2:
        print("need at least 2 sessions per class for a session-wise evaluation", file=sys.stderr)
        return 1
    print(f"{len(y)} windows, {len(set(groups))} sessions, {len(classes)} classes, "
          f"{len(folds)} folds (one held-out session per class per fold), depth<={args.depth}, min-leaf {args.min_leaf}")

    total = np.zeros((len(classes), len(classes)), dtype=np.int64)
    accs = []
    for k, (tr, te) in enumerate(folds):
        t = Tree(args.depth, args.min_leaf).fit(X[tr], y[tr], len(classes))
        p = t.predict(X[te])
        acc = float((p == y[te]).mean())
        accs.append(acc)
        total += confusion(y[te], p, len(classes))
        print(f"  fold {k}: {len(set(groups[te]))} held-out sessions, {len(te)} windows, accuracy {acc:.3f} "
              f"({t.n_leaves()} leaves)")
    print(f"accuracy by session: mean {np.mean(accs):.3f}  min {np.min(accs):.3f}  max {np.max(accs):.3f}")
    print("confusion matrix (all held-out windows):")
    print_confusion(total, classes)

    tree = Tree(args.depth, args.min_leaf).fit(X, y, len(classes))
    print(f"final tree on all sessions: {tree.n_nodes()} nodes, {tree.n_leaves()} leaves, depth {tree.depth()}")
    imp = tree.gain / tree.gain.sum() if tree.gain.sum() > 0 else tree.gain
    for i in np.argsort(-imp)[:5]:
        if imp[i] > 0:
            print(f"  feature {names[i]:10s} importance {imp[i]:.3f}")
    if args.model:
        with open(args.model, "w", encoding="utf-8") as f:
            json.dump(tree.to_dict(names, classes), f, indent=1)
        print(f"model -> {args.model}")
    if args.export_c:
        with open(args.export_c, "w", encoding="utf-8") as f:
            f.write(tree.to_c(names, classes))
        print(f"C header -> {args.export_c}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
