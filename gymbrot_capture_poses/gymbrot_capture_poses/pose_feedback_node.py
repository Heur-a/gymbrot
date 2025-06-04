import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import mxnet as mx
import gluoncv
from gluoncv.model_zoo import get_model
from gluoncv.utils import try_import_cv2
from gluoncv.data.transforms.pose import detector_to_alpha_pose, heatmap_to_coord
import numpy as np
import os
import pandas as pd
from std_msgs.msg import String
from rclpy.qos import qos_profile_sensor_data


cv2 = try_import_cv2()

COCO_17_KEYPOINT_INDICES = list(range(17))

# KEYPOINTS ESPECÍFICOS PARA EXTENSIÓN DE CUÁDRICEPS
LEG_KEYPOINTS = {
    'left_hip': 11,      # Cadera izquierda
    'right_hip': 12,     # Cadera derecha  
    'left_knee': 13,     # Rodilla izquierda
    'right_knee': 14,    # Rodilla derecha
    'left_ankle': 15,    # Talón izquierdo (CLAVE)
    'right_ankle': 16    # Talón derecho (CLAVE)
}

# Índices de los keypoints que nos interesan (solo piernas)
RELEVANT_KEYPOINTS = list(LEG_KEYPOINTS.values())

def select_best_person(pred_coords, confidence):
    avg_conf = confidence.mean(axis=1).asnumpy().flatten()
    best_idx = np.argmax(avg_conf)
    return pred_coords[best_idx].asnumpy(), confidence[best_idx].asnumpy()

def process_frame(frame, detector, estimator, ctx):
    
    nullframes = None, None, None
    height, width = frame.shape[:2]
    frame_rgb_np = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    frame_rgb = mx.nd.array(frame_rgb_np)
    x, frame_processed = gluoncv.data.transforms.presets.ssd.transform_test(frame_rgb, short=240)
    x = x.as_in_context(ctx)

    class_IDs, scores, bounding_boxes = detector(x)
    n_persons = int((scores > 0.5).sum().asscalar())
    if n_persons == 0:
        return nullframes

    pose_input, upscale_bbox = detector_to_alpha_pose(frame_processed, class_IDs, scores, bounding_boxes)
    if upscale_bbox is None:
        return nullframes

    predicted_heatmap = estimator(pose_input.as_in_context(ctx))
    pred_coords, confidence = heatmap_to_coord(predicted_heatmap, upscale_bbox)
    pred_coords, confidence = select_best_person(pred_coords, confidence)

    return pred_coords, confidence, (height, width)

def calculate_leg_angles(coords, conf):
    """
    Calcula los ángulos específicos de las piernas para extensión de cuádriceps
    """
    angles = {}
    
    try:
        # Umbral de confianza más bajo para detectar más casos
        leg_conf_threshold = 0.2  
        
        # Ángulo de rodilla izquierda (cadera-rodilla-talón)
        if (conf[LEG_KEYPOINTS['left_hip']] > leg_conf_threshold and 
            conf[LEG_KEYPOINTS['left_knee']] > leg_conf_threshold and 
            conf[LEG_KEYPOINTS['left_ankle']] > leg_conf_threshold):
            
            hip = coords[LEG_KEYPOINTS['left_hip']]
            knee = coords[LEG_KEYPOINTS['left_knee']]  
            ankle = coords[LEG_KEYPOINTS['left_ankle']]
            
            angle = calculate_angle_3_points(hip, knee, ankle)
            if angle > 0:  # Solo ángulos válidos
                angles['left_leg'] = angle
        
        # Ángulo de rodilla derecha (cadera-rodilla-talón)  
        if (conf[LEG_KEYPOINTS['right_hip']] > leg_conf_threshold and
            conf[LEG_KEYPOINTS['right_knee']] > leg_conf_threshold and
            conf[LEG_KEYPOINTS['right_ankle']] > leg_conf_threshold):
            
            hip = coords[LEG_KEYPOINTS['right_hip']]
            knee = coords[LEG_KEYPOINTS['right_knee']]
            ankle = coords[LEG_KEYPOINTS['right_ankle']]
            
            angle = calculate_angle_3_points(hip, knee, ankle)
    
            if angle > 0:  # Solo ángulos válidos
                angles['right_leg'] = angle
            
        # Calcular ángulo del tronco (para detectar flexión hacia adelante)
        if (conf[LEG_KEYPOINTS['left_hip']] > leg_conf_threshold and
            conf[LEG_KEYPOINTS['right_hip']] > leg_conf_threshold):
            
            # Calcular inclinación del tronco respecto a la vertical
            hip_center = (coords[LEG_KEYPOINTS['left_hip']] + coords[LEG_KEYPOINTS['right_hip']]) / 2
            
            # Vector vertical de referencia (hacia abajo)
            vertical_ref = np.array([0, 100])  # 100 píxeles hacia abajo
            vertical_point = hip_center + vertical_ref
            
            # Obtener un punto superior del cuerpo (aproximado)
            # Usamos un punto arriba de las caderas como referencia del tronco
            trunk_top = hip_center - np.array([0, 50])  # 50 píxeles arriba
            
            trunk_angle = calculate_angle_3_points(trunk_top, hip_center, vertical_point)
            angles['trunk_inclination'] = trunk_angle
            
        # Elevación de talones
        if (conf[LEG_KEYPOINTS['left_ankle']] > leg_conf_threshold and
            conf[LEG_KEYPOINTS['right_ankle']] > leg_conf_threshold and
            conf[LEG_KEYPOINTS['left_hip']] > leg_conf_threshold and  
            conf[LEG_KEYPOINTS['right_hip']] > leg_conf_threshold):
            
            # Calcular altura promedio de caderas como referencia
            hip_avg_y = (coords[LEG_KEYPOINTS['left_hip']][1] + coords[LEG_KEYPOINTS['right_hip']][1]) / 2
            
            # Calcular altura promedio de talones
            ankle_avg_y = (coords[LEG_KEYPOINTS['left_ankle']][1] + coords[LEG_KEYPOINTS['right_ankle']][1]) / 2
            
            # Elevación relativa normalizada por altura de imagen
            elevation = (hip_avg_y - ankle_avg_y) / 100.0  # Normalizar por altura típica
            angles['heel_elevation'] = elevation
            
    except Exception as e:
        print(f"[DEBUG] Error calculando ángulos: {e}")
        
    return angles

def calculate_angle_3_points(p1, p2, p3):
    """
    Calcula el ángulo en p2 formado por los puntos p1-p2-p3
    """
    try:
        # Vectores desde p2 hacia p1 y p3
        v1 = np.array(p1) - np.array(p2)
        v2 = np.array(p3) - np.array(p2)
        
        # Verificar que los vectores no sean cero
        if np.linalg.norm(v1) == 0 or np.linalg.norm(v2) == 0:
            return 0.0
        
        # Calcular ángulo usando producto punto
        cos_angle = np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2))
        cos_angle = np.clip(cos_angle, -1, 1)
        angle = np.arccos(cos_angle)
        
        return np.degrees(angle)
    
    except Exception as e:
        print(f"[DEBUG] Error en cálculo de ángulo: {e}")
        return 0.0

def classify_quadriceps_phase(coords, conf, image_shape):
    """
    Clasifica la fase de extensión de cuádriceps con lógica mejorada
    """
    angles = calculate_leg_angles(coords, conf)
    
    if not angles:
        return None, 0.0, "No se pudieron calcular ángulos de piernas"
    
    # Obtener ángulos de ambas piernas si están disponibles
    leg_angles = []
    if 'left_leg' in angles:
        leg_angles.append(angles['left_leg'])
    if 'right_leg' in angles:
        leg_angles.append(angles['right_leg'])
        
    if not leg_angles:
        return None, 0.0, "No se detectaron ángulos de piernas válidos"
    
    avg_leg_angle = np.mean(leg_angles)
    heel_elevation = angles.get('heel_elevation', 0)
    trunk_inclination = angles.get('trunk_inclination', 90)  # 90° es vertical
    
        # --- LÓGICA DE CLASIFICACIÓN ---

    phase = None
    confidence = 0.0
    details = ""

    # Factores para determinar la fase, SOLO basados en ángulo de piernas e inclinación del tronco
    is_legs_flexed = avg_leg_angle < 130
    is_legs_extended = avg_leg_angle > 150
    is_legs_intermediate = 130 <= avg_leg_angle <= 150

    is_trunk_forward = trunk_inclination < 80
    is_trunk_upright = trunk_inclination >= 80

    # CLASIFICACIÓN POR FASES (sin talones)
    if is_legs_flexed and is_trunk_upright:
        phase = "inicio"
        confidence = 0.85
        details = f"Posición inicial: piernas flexionadas ({avg_leg_angle:.1f}°)"
        if avg_leg_angle < 120:
            confidence += 0.05

    elif is_legs_intermediate:
        phase = "medio"
        confidence = 0.8
        details = f"Fase de transición: piernas en extensión ({avg_leg_angle:.1f}°)"
        if 135 <= avg_leg_angle <= 145:
            confidence += 0.05

    elif is_legs_extended and is_trunk_upright:
        phase = "final"
        confidence = 0.85
        details = f"Extensión completa: piernas extendidas ({avg_leg_angle:.1f}°)"
        if avg_leg_angle > 160:
            confidence += 0.05

    else:
        # CASOS AMBIGUOS
        if avg_leg_angle < 135:
            phase = "inicio"
            confidence = 0.6
            details = f"Posición inicial probable ({avg_leg_angle:.1f}°) - forma ambigua"
        elif avg_leg_angle > 140:
            phase = "final"
            confidence = 0.6
            details = f"Extensión probable ({avg_leg_angle:.1f}°) - forma ambigua"
        else:
            phase = "medio"
            confidence = 0.6
            details = f"Transición probable ({avg_leg_angle:.1f}°) - forma ambigua"

    # Penalización por mala postura del tronco
    if is_trunk_forward:
        confidence -= 0.1
        details += " (⚠️ tronco inclinado)"

    # Penalización por asimetría de piernas
    if len(leg_angles) == 2:
        angle_diff = abs(leg_angles[0] - leg_angles[1])
        if angle_diff > 15:
            confidence -= 0.1
            details += " (⚠️ asimetría)"

    # Observación complementaria sobre los talones (sin afectar fase/confianza)
    if 'heel_elevation' in angles:
        heel_elevation = angles['heel_elevation']
        if heel_elevation > 0.5:
            details += " | Observación: talones muy elevados"
        elif heel_elevation < -0.2:
            details += " | Observación: talones muy bajos"
        elif -0.1 <= heel_elevation <= 0.3:
            details += " | Observación: talones en posición neutra"

    confidence = max(0.0, min(1.0, confidence))
    return phase, confidence, details


def extract_leg_features(coords, conf, image_shape):
    """
    Extrae características específicas de las piernas para comparación
    """
    # Umbral de confianza más bajo
    leg_conf_threshold = 0.2  
    features = []
    
    # Normalizar coordenadas por tamaño de imagen
    h, w = image_shape
    normalized_coords = coords / np.array([w, h])
    
    # Extraer posiciones de keypoints de piernas
    for kp_name, kp_idx in LEG_KEYPOINTS.items():
        if conf[kp_idx] > leg_conf_threshold:
            features.extend([normalized_coords[kp_idx][0], normalized_coords[kp_idx][1]])
        else:
            features.extend([0.0, 0.0])  # Placeholder si no hay confianza
    
    # Agregar ángulos calculados como características
    angles = calculate_leg_angles(coords, conf)
    
    # Normalizar ángulos a rango 0-1
    if 'left_leg' in angles:
        features.append(angles['left_leg'] / 180.0)
    else:
        features.append(0.0)
        
    if 'right_leg' in angles:
        features.append(angles['right_leg'] / 180.0) 
    else:
        features.append(0.0)
        
    if 'heel_elevation' in angles:
        # Normalizar elevación (puede ser negativa o positiva)
        features.append(max(-1.0, min(1.0, angles['heel_elevation'])))
    else:
        features.append(0.0)
        
    # Normalizar inclinación del tronco. Rango 0-1 (0° a 180°)
    # Asumimos que el tronco está vertical a 90° (0° es horizontal)
    # 0° = 0.0, 90° = 0.5, 180° = 1.0
    if 'trunk_inclination' in angles:
        features.append(angles['trunk_inclination'] / 180.0)
    else:
        features.append(0.5)  # Valor neutro (90° normalizado)
        
    return np.array(features)

def calculate_quadriceps_similarity(coords1, conf1, coords2, conf2, image_shape):
    """
    Calcula similitud específica para extensión de cuádriceps con mejor sensibilidad
    """
    # Extraer características de piernas
    features1 = extract_leg_features(coords1, conf1, image_shape)
    features2 = extract_leg_features(coords2, conf2, image_shape)
    
    if len(features1) == 0 or len(features2) == 0:
        return 0.0
    
    # Calcular distancia euclidea entre características
    distance = np.linalg.norm(features1 - features2)
    
    
    if distance < 0.05:  # Muy similar
        similarity = 0.95 - (distance * 4)
    elif distance < 0.15:  # Moderadamente similar  
        similarity = 0.85 - (distance * 3)
    elif distance < 0.3:  # Algo diferente
        similarity = 0.7 - (distance * 2)
    elif distance < 0.5:  # Bastante diferente
        similarity = 0.5 - (distance * 1)
    else:  # Muy diferente
        similarity = 0.2 - (distance * 0.3)
    
    return max(0.0, min(1.0, similarity))

def load_keypoints_dataset(csv_path):
    """Carga dataset optimizado para extensión de cuádriceps"""
    if not os.path.exists(csv_path):
        raise FileNotFoundError(f"No se encontró el archivo CSV: {csv_path}")

    try:
        df = pd.read_csv(csv_path)
    except Exception as e:
        print(f"[ERROR] Error leyendo CSV: {e}")
        raise

    if 'label' not in df.columns:
        raise ValueError("El CSV debe contener una columna 'label'")

    labels = df['label'].unique()
    reference_poses = {}

    for label in labels:
        
        label_rows = df[df['label'] == label]
        keypoint_cols = [col for col in df.columns if col.startswith('kp_')]
        
        if len(keypoint_cols) != 66:
            raise ValueError(f"Se esperaban 66 columnas kp_*, encontradas {len(keypoint_cols)}")
        
        keypoint_data = label_rows[keypoint_cols].values
        keypoints_reshaped = keypoint_data.reshape(keypoint_data.shape[0], 33, 2)
        keypoints_coco17 = keypoints_reshaped[:, COCO_17_KEYPOINT_INDICES, :]
        
        # Calcular pose promedio
        mean_pose = keypoints_coco17.mean(axis=0)
        
        # Generar confianza basada en variabilidad de keypoints de piernas
        if keypoints_coco17.shape[0] > 1:
            leg_keypoints_data = keypoints_coco17[:, RELEVANT_KEYPOINTS, :]
            leg_variance = leg_keypoints_data.var(axis=0).mean(axis=1)
            
            # Inicializar confianza base más alta
            confidence = np.ones(mean_pose.shape[0]) * 0.6  
            
            # Asignar alta confianza a keypoints de piernas
            for kp_idx in RELEVANT_KEYPOINTS:
                confidence[kp_idx] = 0.85 
            
            # Ajustar por variabilidad (menos penalización)
            for i, kp_idx in enumerate(RELEVANT_KEYPOINTS):
                var_penalty = min(0.2, leg_variance[i] * 0.0008)  
                confidence[kp_idx] -= var_penalty
        else:
            confidence = np.ones(mean_pose.shape[0]) * 0.6
            # Dar extra confianza a keypoints de piernas
            for kp_idx in RELEVANT_KEYPOINTS:
                confidence[kp_idx] = 0.85
        
        confidence = np.clip(confidence, 0.2, 1.0)  
        reference_poses[label] = (mean_pose, confidence)
        
        # Mostrar ángulos promedio para esta clase
        sample_angles = calculate_leg_angles(mean_pose, confidence)

    return reference_poses

class PoseComparisonNode(Node):
    def __init__(self):
        super().__init__('quadriceps_extension_classifier')

        self.bridge = CvBridge()
        self.ctx = mx.cpu()

        self.get_logger().info("Cargando modelo detector...")
        self.detector = get_model('ssd_512_mobilenet1.0_coco', pretrained=True, ctx=self.ctx)
        self.detector.reset_class(classes=['person'], reuse_weights={'person': 'person'})

        self.get_logger().info("Cargando modelo de pose...")
        self.estimator = get_model('alpha_pose_resnet101_v1b_coco', pretrained=True, ctx=self.ctx)

        csv_path = 'src/gymbrot/rosweb/assets/dataset_ejercicios/csv_output/pose_keypoints_dataset.csv'
        
        try:
            self.references = load_keypoints_dataset(csv_path)
            self.get_logger().info(f"Referencias cargadas: {list(self.references.keys())}")
        except Exception as e:
            self.get_logger().error(f"Error cargando referencias: {e}")
            raise

        # Añadir variables para seguimiento
        self.last_phase = None
        self.phase_counter = 0
        self.repetition_count = 0
        self.phase_confidence_threshold = 0.7
        self.consecutive_frames_threshold = 3
        self.same_phase_counter = 0

        self.subscription = self.create_subscription(
            Image,
            '/image',
            self.image_callback,
            qos_profile_sensor_data,
        )
        self.pose_feedback_pub = self.create_publisher(String, '/pose_feedback', 10)
        self.pose_image_pub = self.create_publisher(Image, '/pose', 10)
        self.bridge = CvBridge()

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f"Error al convertir imagen ROS a OpenCV: {e}")
            return

        self.process_quadriceps_exercise(frame)

    def process_quadriceps_exercise(self, frame):
        coords, conf, shape = process_frame(frame, self.detector, self.estimator, self.ctx)
        # Iniciar la variable desde el principio
        full_feedback = " "
        if coords is None:
            self.get_logger().info("No se detectó persona en la imagen")
            return

        # MÉTODO 1: Clasificación basada en ángulos
        phase_angle, conf_angle, details = classify_quadriceps_phase(coords, conf, shape)
        
        # MÉTODO 2: Comparación con referencias del dataset
        similarities = {}
        for label, (ref_coords, ref_conf) in self.references.items():
            sim = calculate_quadriceps_similarity(ref_coords, ref_conf, coords, conf, shape)
            similarities[label] = sim

        best_dataset_label = max(similarities, key=similarities.get) if similarities else None
        best_dataset_sim = similarities.get(best_dataset_label, 0) if best_dataset_label else 0

        # Seguimiento de fases y repeticiones
        if phase_angle and conf_angle > self.phase_confidence_threshold:
            if phase_angle == self.last_phase:
                self.same_phase_counter += 1
            else:
                self.same_phase_counter = 1
                
            if self.same_phase_counter >= self.consecutive_frames_threshold:
                if self.last_phase != phase_angle:
                    self.phase_counter += 1
                    if phase_angle == "inicio" and self.last_phase == "final":
                        self.repetition_count += 1
                    self.last_phase = phase_angle
        full_feedback += str(phase_angle)
        # RESULTADO
        self.get_logger().info("\n" + "=" * 60)
        self.get_logger().info("ANÁLISIS DE EXTENSIÓN DE CUÁDRICEPS:")
        self.get_logger().info(f"   {details}")
        
        if phase_angle:
            self.get_logger().info(f"FASE ACTUAL: {phase_angle.upper()} ({conf_angle*100:.1f}%)")
            self.get_logger().info(f"Repeticiones completadas: {self.repetition_count}")
            
            if best_dataset_sim > 0.4:
                self.get_logger().info(f"Comparación con dataset: {best_dataset_label} ({best_dataset_sim*100:.1f}%)")
        
        # Evaluación de calidad
        if phase_angle and conf_angle > 0.5:
            if conf_angle > 0.9:
                calidad = "✅ ¡Excelente ejecución!"
            elif conf_angle > 0.85:
                calidad = "✅ ¡Buena ejecución!"
            elif conf_angle > 0.65:
                calidad = "⚠️  Ejecución aceptable"
            else:
                calidad = "⚠️  Necesita mejorar la técnica"
            full_feedback += calidad + "\n"
        else:
            calidad = "Confianza insuficiente para evaluación"
        self.get_logger().info(full_feedback)

    # Publicar todo el mensaje en /pose_feedback
        feedback_msg = String()
        feedback_msg.data = full_feedback
        self.pose_feedback_pub.publish(feedback_msg)
        # Publicar imagen en /pose
        try:
            image_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.pose_image_pub.publish(image_msg)
        except Exception as e:
            self.get_logger().error(f"Error al convertir/publicar la imagen: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = PoseComparisonNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()