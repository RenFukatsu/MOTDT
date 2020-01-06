import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from cv_bridge import CvBridgeError
import message_filters

from models.classification.classifier import PatchClassifier
from models.reid import load_reid_model, extract_reid_features

class HumanClassifier(Node):

    def __init__(self, min_cls_score=0.4, use_refind=True):
        super().__init__("motdt_classifier")
        self.image_sub = message_filters.Subscriber('/usb_cam/image_raw', Image)
        self.bboxes_sub = message_filters.Subscriber('/yolo/bboxes', BBox)
        self.time_synchronizer = message_filters.TimeSynchronizer([self.image_sub, self.bboxes_sub], 10)
        self.time_synchronizer.registerCallback(self.callback)
        self.bridge = CvBridge()
        self.min_cls_score = min_cls_score
        self.use_refind = use_refind
        self.classifier = PatchClassifier()
        self.reid_model = load_reid_model()
    
    def callback(self, image, bboxes):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(image, 'rgb8')
        except CvBridgeError as e:
            print(e)

    
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


def main(args=None):
    rclpy.init(args=args)
    human_classifier = HumanClassifier()
    rclpy.spin(human_classifier)
    human_classifier.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
