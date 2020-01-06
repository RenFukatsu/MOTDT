import rclpy
from rclpy.node import Node
import numpy as np

from sensor_msgs.msg import Image

from models.classification.classifier import PatchClassifier
from models.reid import load_reid_model, extract_reid_features

class HumanClassifier():

    def __init__(self, min_cls_score=0.4, use_refind=True):
        self.min_cls_score = min_cls_score
        self.use_refind = use_refind
        self.classifier = PatchClassifier()
        self.reid_model = load_reid_model()
    
    def get_scores(self, image, tlwhs, det_scores):
        self.classifier.update(image)
        tlbrs = []
        for tlwh in tlwhs:
            tlbrs.append(tlwh[2:] + tlwh[:2])
        rois = np.asarray([tlbr for tlbr in tlbrs], dtype=np.float32)

        cls_scores = self.classifier.predict(rois)
        scores = np.asarray([det_score for det_score in det_scores], dtype=np.float)
        scores = scores * cls_scores

        return scores
    
    def get_features(self, image, tlbrs):
        features = extract_reid_features(self.reid_model, image, tlbrs)
        features = features.cpu().numpy()

        return features





