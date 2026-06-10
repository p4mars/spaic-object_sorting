import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from std_msgs.msg import String

import cv2
import numpy as np
from ultralytics import YOLO
import math
import time

def cluster_detections(detections, radius=10):
    clusters = []

    for det in detections:
        u, v = det["pixel"]
        added = False

        # Try to add detection to an existing cluster
        for cluster in clusters:
            for other in cluster:
                ou, ov = other["pixel"]
                if (u - ou)**2 + (v - ov)**2 <= radius**2:
                    cluster.append(det)
                    added = True
                    break
            if added:
                break

        # If not added to any cluster, create a new one
        if not added:
            clusters.append([det])

    return clusters

def fuse_clusters(clusters):
    fused = []

    for cluster in clusters:
        n = len(cluster)

        # average position
        xs = [d["pos"][0] for d in cluster]
        ys = [d["pos"][1] for d in cluster]
        zs = [d["pos"][2] for d in cluster]
        pos = [
            sum(xs) / n,
            sum(ys) / n,
            sum(zs) / n
        ]

        # average confidence
        confs = [d["conf"] for d in cluster]
        conf = sum(confs) / n

        # majority class
        classes = [d["class"] for d in cluster]
        class_final = max(set(classes), key=classes.count)

        fused.append({
            "pos": pos,
            "conf": conf,
            "class": class_final,
            "occurrences": len(cluster)
        })

    return fused

def pick_best(fused):
    scores = []
    for fuse in fused:
        x, y, z = fuse["pos"]
        dist = np.sqrt(x**2 + y**2 + z**2)
        conf = fuse["conf"]
        occ = fuse["occurrences"]
        
        # weights
        w_d = 4
        w_c = 1
        w_o = 3

        # distance should be minimized, confidence/occurrences maximized
        score = (w_d * (1.0 / (dist + 1e-6))) + w_c * conf + w_o * occ
        scores.append(score)

    if not scores:
        return None

    best_idx = int(np.argmax(scores))
    return fused[best_idx]   # <-- return full fused dict now (pos + class)