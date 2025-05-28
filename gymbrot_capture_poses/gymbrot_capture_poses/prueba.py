import mxnet as mx
import gluoncv
from gluoncv.model_zoo import get_model
from gluoncv.utils import try_import_cv2
from gluoncv.data.transforms.pose import detector_to_alpha_pose, heatmap_to_coord
from gluoncv.utils.viz import cv_plot_keypoints
import numpy as np

cv2 = try_import_cv2()

def filter_bboxes(class_IDs, scores, bounding_boxes, score_thresh=0.5):
    # class_IDs, scores: (1, N)
    # bounding_boxes: (1, N, 4)

    class_IDs_np = class_IDs.asnumpy().reshape(-1)  # (N,)
    scores_np = scores.asnumpy().reshape(-1)        # (N,)
    bounding_boxes_np = bounding_boxes.asnumpy().squeeze(axis=0)  # (N,4)

    mask = (scores_np > score_thresh) & (class_IDs_np == 0)
    if np.sum(mask) == 0:
        return None, None, None

    filtered_class_IDs = mx.nd.array(class_IDs_np[mask], ctx=class_IDs.context).expand_dims(axis=0)  # (1, M)
    filtered_scores = mx.nd.array(scores_np[mask], ctx=scores.context).expand_dims(axis=0)          # (1, M)
    filtered_bboxes = mx.nd.array(bounding_boxes_np[mask], ctx=bounding_boxes.context).expand_dims(axis=0)  # (1, M, 4)

    return filtered_class_IDs, filtered_scores, filtered_bboxes


def load_pose_from_image(image_path, detector, estimator, ctx, score_thresh=0.5):
    frame = cv2.imread(image_path)
    if frame is None:
        raise FileNotFoundError(f"No se pudo leer la imagen: {image_path}")

    frame_rgb_np = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    frame_rgb = mx.nd.array(frame_rgb_np)

    x, frame_processed = gluoncv.data.transforms.presets.ssd.transform_test(frame_rgb, short=256)
    x = x.as_in_context(ctx)
    class_IDs, scores, bounding_boxes = detector(x)

    # Filtrar detecciones para persona con threshold
    class_IDs, scores, bounding_boxes = filter_bboxes(class_IDs, scores, bounding_boxes, score_thresh)
    if class_IDs is None:
        print(f"[INFO] No se detectaron personas con suficiente confianza en {image_path}")
        return None, None, frame.shape[:2], None, None

    pose_input, upscale_bbox = detector_to_alpha_pose(frame_processed, class_IDs, scores, bounding_boxes)

    if upscale_bbox is not None:
        predicted_heatmap = estimator(pose_input.as_in_context(ctx))
        pred_coords, confidence = heatmap_to_coord(predicted_heatmap, upscale_bbox)
        return pred_coords[0].asnumpy(), confidence[0].asnumpy(), frame.shape[:2], frame_processed, bounding_boxes[0][0].asnumpy()
    else:
        print(f"[INFO] No se detectaron personas en la imagen {image_path}")
        return None, None, frame.shape[:2], None, None

def draw_pose(frame_processed, coords, conf, bbox, out_path="pose_output.jpg"):
    pred_coords = mx.nd.array([coords])
    confidence = mx.nd.array([conf])
    dummy_ids = mx.nd.array([[[0]]])  # 3D: (1, 1, 1)
    dummy_scores = mx.nd.array([[[1]]])  # 3D: (1, 1, 1)
    # Convierte bbox a formato esperado (1,1,4)
    boxes = mx.nd.array(bbox).reshape((1,1,4))

    img = cv_plot_keypoints(frame_processed, pred_coords, confidence,
                            dummy_ids, boxes, dummy_scores,
                            box_thresh=0.5, keypoint_thresh=0.2)
    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    cv2.imwrite(out_path, img)
    print(f"[INFO] Esqueleto guardado en {out_path}")

def normalize_keypoints(coords, image_shape):
    h, w = image_shape
    return coords / np.array([w, h])

def calculate_pose_similarity(coords1, conf1, coords2, conf2, image_shape):
    assert coords1.shape == coords2.shape

    valid = (conf1.ravel() > 0.1) & (conf2.ravel() > 0.1)
    print(f"[DEBUG] Puntos válidos para comparar: {valid.sum()} de {len(valid)}")

    if valid.sum() == 0:
        print("[WARN] No hay puntos con suficiente confianza en ambas imágenes.")
        return 0.0

    coords1_valid = normalize_keypoints(coords1[valid], image_shape)
    coords2_valid = normalize_keypoints(coords2[valid], image_shape)

    distances = np.linalg.norm(coords1_valid - coords2_valid, axis=1)
    mean_distance = distances.mean()

    similarity = max(0.0, 1.0 - mean_distance)
    return similarity

def main():
    ctx = mx.cpu()
    
    print("[INFO] Cargando modelos...")
    detector = get_model('ssd_512_mobilenet1.0_coco', pretrained=True, ctx=ctx)
    detector.reset_class(classes=['person'], reuse_weights={'person': 'person'})
    
    estimator = get_model('simple_pose_resnet18_v1b', pretrained=True, ctx=ctx)
    
    image1_path = "fotos/frame_3.png"
    image2_path = "fotos/frame_0068.png"
    
    coords1, conf1, shape1, frame_proc1, bbox1 = load_pose_from_image(image1_path, detector, estimator, ctx)
    coords2, conf2, shape2, frame_proc2, bbox2 = load_pose_from_image(image2_path, detector, estimator, ctx)

    if coords1 is None or coords2 is None:
        print("[ERROR] No se detectaron personas en una o ambas imágenes.")
        return

    draw_pose(frame_proc1, coords1, conf1, bbox1, "pose1.jpg")
    draw_pose(frame_proc2, coords2, conf2, bbox2, "pose2.jpg")

    similarity = calculate_pose_similarity(coords1, conf1, coords2, conf2, image_shape=shape1)
    print(f"[RESULTADO] Índice de similitud de pose: {similarity * 100:.2f}%")

if __name__ == "__main__":
    main()
