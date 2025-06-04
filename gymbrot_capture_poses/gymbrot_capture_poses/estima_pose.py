import mxnet as mx
from gluoncv import model_zoo, data
from gluoncv.data.transforms.pose import detector_to_alpha_pose, heatmap_to_coord
from gluoncv.utils.viz import cv_plot_keypoints
import cv2
import numpy as np

class PoseEstimator:
    def __init__(self, ctx=mx.cpu()):
        self.ctx = ctx

        # Cargar detector de personas
        self.detector = model_zoo.get_model('ssd_512_mobilenet1.0_coco', pretrained=True, ctx=self.ctx)
        self.detector.reset_class(classes=['person'], reuse_weights={'person': 'person'})

        # Cargar modelo de estimación de pose
        self.estimator = model_zoo.get_model('alpha_pose_resnet101_v1b_coco', pretrained=True, ctx=self.ctx)

    def estimate(self, frame, return_vis=False):
        # Convertir BGR (OpenCV) a RGB
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        frame_nd = mx.nd.array(frame_rgb).astype('uint8')

        # Preprocesamiento para el detector
        x, img = data.transforms.presets.ssd.transform_test(frame_nd, short=512)
        x = x.as_in_context(self.ctx)

        # Ejecutar detector
        class_IDs, scores, bounding_boxes = self.detector(x)
        class_IDs = class_IDs[0]
        scores = scores[0]
        bounding_boxes = bounding_boxes[0]

        # Filtrar detecciones con score > 0.5
        valid = (scores > 0.5)
        np_indices = np.nonzero(valid.asnumpy())[0]

        if len(np_indices) == 0:
            if return_vis:
                return None, None, frame
            else:
                return None, None, None

        indices = mx.nd.array(np_indices, dtype='int32')

        class_IDs = mx.nd.take(class_IDs, indices)
        scores = mx.nd.take(scores, indices)
        bounding_boxes = mx.nd.take(bounding_boxes, indices)

        # Preparar entrada para estimador de pose
        pose_input, upscale_bbox = detector_to_alpha_pose(img, class_IDs, scores, bounding_boxes)

        if pose_input is None:
            if return_vis:
                return None, None, frame
            else:
                return None, None, None

        # Ejecutar estimación de pose
        predicted_heatmap = self.estimator(pose_input.as_in_context(self.ctx))
        pred_coords, confidence = heatmap_to_coord(predicted_heatmap, upscale_bbox)

        # Visualización opcional
        if return_vis:
            vis_img = cv_plot_keypoints(img, pred_coords, confidence, class_IDs, bounding_boxes, scores, keypoint_thresh=0.2)
            vis_img_bgr = cv2.cvtColor(vis_img, cv2.COLOR_RGB2BGR)
            return pred_coords, confidence, vis_img_bgr

        return pred_coords, confidence, None
