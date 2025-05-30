import mxnet as mx
import os
import gluoncv
from gluoncv.model_zoo import get_model
from gluoncv.utils import try_import_cv2
from gluoncv.data.transforms.pose import detector_to_alpha_pose, heatmap_to_coord
from gluoncv.utils.viz import cv_plot_keypoints
import numpy as np

cv2 = try_import_cv2()

def select_best_person(pred_coords, confidence):
    avg_conf = confidence.mean(axis=1).asnumpy().flatten()
    best_idx = np.argmax(avg_conf)
    return pred_coords[best_idx].asnumpy(), confidence[best_idx].asnumpy()

def process_image(image_path, detector, estimator, ctx):
    frame = cv2.imread(image_path)
    if frame is None:
        print(f"[ERROR] No se pudo leer la imagen: {image_path}")
        return None, None, None

    height, width = frame.shape[:2]
    print(f"[INFO] Procesando {image_path} con tamaño {width}x{height}")

    frame_rgb_np = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    frame_rgb = mx.nd.array(frame_rgb_np)

    x, frame_processed = gluoncv.data.transforms.presets.ssd.transform_test(frame_rgb, short=240)
    x = x.as_in_context(ctx)

    class_IDs, scores, bounding_boxes = detector(x)
    n_persons = int((scores > 0.5).sum().asscalar())
    print(f"[INFO] Detecciones con confianza > 0.5: {n_persons} personas")

    pose_input, upscale_bbox = detector_to_alpha_pose(frame_processed, class_IDs, scores, bounding_boxes)

    if upscale_bbox is not None:
        predicted_heatmap = estimator(pose_input.as_in_context(ctx))
        pred_coords, confidence = heatmap_to_coord(predicted_heatmap, upscale_bbox)

        # Elegir persona con mayor confianza promedio
        pred_coords, confidence = select_best_person(pred_coords, confidence)

        img = cv_plot_keypoints(frame_processed, mx.nd.array([pred_coords]), mx.nd.array([confidence]), class_IDs, bounding_boxes, scores, box_thresh=0.5, keypoint_thresh=0.2)
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        out_path = os.path.splitext(os.path.basename(image_path))[0] + "_output.jpg"
        cv2.imwrite(out_path, img)
        print(f"[INFO] Imagen guardada como {out_path}")
        return pred_coords, confidence, (height, width)
    else:
        print("[INFO] No se detectaron personas en la imagen.")
        return None, None, None

def normalize_keypoints(coords, image_shape):
    h, w = image_shape
    return coords / np.array([w, h])

def calculate_pose_similarity(coords1, conf1, coords2, conf2, image_shape):
    assert coords1.shape == coords2.shape

    # Asegurarse de que confianzas están en forma (17,)
    conf1 = conf1.flatten()
    conf2 = conf2.flatten()

    # Subir el umbral de confianza
    valid = (conf1 > 0.3) & (conf2 > 0.3)
    print(f"[DEBUG] Puntos válidos para comparar: {valid.sum()} de {len(valid)}")

    if valid.sum() == 0:
        print("[WARN] No hay puntos con suficiente confianza en ambas imágenes.")
        return 0.0

    coords1_valid = normalize_keypoints(coords1[valid], image_shape)
    coords2_valid = normalize_keypoints(coords2[valid], image_shape)

    distances = np.linalg.norm(coords1_valid - coords2_valid, axis=1)

    # Mostrar distancia por punto
    print("[DEBUG] Distancias por punto:")
    for i, d in zip(np.where(valid)[0], distances):
        print(f"Punto {i}: distancia normalizada = {d:.4f}")

    mean_distance = distances.mean()
    similarity = max(0.0, 1.0 - mean_distance)
    return similarity

def calculate_lower_body_similarity(coords1, conf1, coords2, conf2, image_shape):
    lower_body_indices = [11, 12, 13, 14, 15, 16]
    
    valid = [(conf1[i] > 0.1 and conf2[i] > 0.1) for i in lower_body_indices]
    if not any(valid):
        print("[WARN] No hay puntos válidos en las piernas.")
        return 0.0

    distances = []
    print("[DEBUG] Distancias (solo parte inferior):")
    for i in lower_body_indices:
        if conf1[i] > 0.1 and conf2[i] > 0.1:
            pt1 = coords1[i] / np.array(image_shape[::-1])
            pt2 = coords2[i] / np.array(image_shape[::-1])
            dist = np.linalg.norm(pt1 - pt2)
            distances.append(dist)
            print(f"Punto {i}: distancia normalizada = {dist:.4f}")

    mean_distance = np.mean(distances)
    similarity = max(0.0, 1.0 - mean_distance)
    return similarity


def main():
    ctx = mx.cpu()

    print("[INFO] Cargando modelo detector...")
    detector = get_model('ssd_512_mobilenet1.0_coco', pretrained=True, ctx=ctx)
    detector.reset_class(classes=['person'], reuse_weights={'person': 'person'})

    print("[INFO] Cargando modelo de pose...")
    estimator = get_model('alpha_pose_resnet101_v1b_coco', pretrained=True, ctx=ctx)

    image_paths = ["../../rosweb/assets/dataset_ejercicios/IMG_7666.png", "fotos/frame_0068.png"]

    results = []
    for image_path in image_paths:
        coords, conf, shape = process_image(image_path, detector, estimator, ctx)
        results.append((coords, conf, shape))

    coords1, conf1, shape1 = results[0]
    coords2, conf2, shape2 = results[1]

    similarity = calculate_pose_similarity(coords1, conf1, coords2, conf2, image_shape=shape1)
    print(f"[RESULTADO] Índice de similitud de pose: {similarity * 100:.2f}%")
    similarity_lower = calculate_lower_body_similarity(coords1, conf1, coords2, conf2, image_shape=shape1)
    print(f"[RESULTADO] Similitud parte inferior del cuerpo: {similarity_lower * 100:.2f}%")

if __name__ == "__main__":
    main()
