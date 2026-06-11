import numpy as np


def cluster_detections(detections, radius=10):
    """Group nearby detections (in pixel space) into clusters."""
    clusters = []
    for det in detections:
        u, v = det["pixel"]
        added = False
        for cluster in clusters:
            for other in cluster:
                ou, ov = other["pixel"]
                if (u - ou) ** 2 + (v - ov) ** 2 <= radius ** 2:
                    cluster.append(det)
                    added = True
                    break
            if added:
                break
        if not added:
            clusters.append([det])
    return clusters


def fuse_clusters(clusters):
    """Average each cluster into a single detection with majority class."""
    fused = []
    for cluster in clusters:
        n = len(cluster)
        xs = [d["pos"][0] for d in cluster]
        ys = [d["pos"][1] for d in cluster]
        zs = [d["pos"][2] for d in cluster]
        pos = [sum(xs) / n, sum(ys) / n, sum(zs) / n]
        conf = sum(d["conf"] for d in cluster) / n
        classes = [d["class"] for d in cluster]
        class_final = max(set(classes), key=classes.count)
        fused.append({
            "pos": pos,
            "conf": conf,
            "class": class_final,
            "occurrences": n,
        })
    return fused


def pick_best(fused):
    """Score each fused detection and return the highest-scoring one."""
    if not fused:
        return None
    scores = []
    for item in fused:
        x, y, z = item["pos"]
        dist = np.sqrt(x ** 2 + y ** 2 + z ** 2)
        score = (4 * (1.0 / (dist + 1e-6))) + 2 * item["conf"] + 1 * item["occurrences"]
        scores.append(score)
    return fused[int(np.argmax(scores))]
