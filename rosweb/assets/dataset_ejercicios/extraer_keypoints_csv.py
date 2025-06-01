import os
import json
import cv2
import mediapipe as mp
import numpy as np
import pandas as pd

# Configuración
input_folder = os.path.expanduser('~/turtlebot3_ws/src/gymbrot/rosweb/assets/dataset_ejercicios')
output_json_folder = './keypoints_json'
output_csv_folder = './csv_output'  # Nueva carpeta para CSV (opcional)

# Crear carpetas de salida antes de empezar
os.makedirs(output_json_folder, exist_ok=True)
os.makedirs(output_csv_folder, exist_ok=True)

mp_pose = mp.solutions.pose
pose = mp_pose.Pose(static_image_mode=True, model_complexity=1)
mp_drawing = mp.solutions.drawing_utils

def infer_label(filename):
    if filename.lower().startswith('ini'):
        return 'inicio'
    elif filename.lower().startswith('mid'):
        return 'medio'
    elif filename.lower().startswith('fin'):
        return 'final'
    return 'desconocido'

# Dataset CSV opcional
dataset_rows = []

# Recorrer subcarpetas (inicio, medio, final)
for subfolder in ['inicio', 'medio', 'final']:
    folder_path = os.path.join(input_folder, subfolder)
    if not os.path.exists(folder_path):
        print(f"⚠️ La carpeta {folder_path} no existe, se omite.")
        continue

    for fname in sorted(os.listdir(folder_path)):
        if not fname.endswith('.png'):
            continue
        fpath = os.path.join(folder_path, fname)
        image = cv2.imread(fpath)

        if image is None:
            print(f"❌ No se pudo leer {fpath}")
            continue

        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        results = pose.process(image_rgb)

        if not results.pose_landmarks:
            print(f"❌ Sin detección de pose en {fname}")
            continue

        landmarks = results.pose_landmarks.landmark
        keypoints = [[lm.x, lm.y] for lm in landmarks]  # O usa lm.visibility si quieres
        flat_keypoints = np.array(keypoints).flatten().tolist()

        # Guardar JSON estilo AlphaPose/OpenPose
        out_data = [{
            'file': fname,
            'label': subfolder,
            'keypoints': flat_keypoints
        }]
        json_name = fname.replace('.png', '_keypoints.json')
        json_path = os.path.join(output_json_folder, json_name)

        with open(json_path, 'w') as f:
            json.dump(out_data, f)

        # También guardar para el CSV final
        row = {'filename': fname, 'label': subfolder}
        for i, val in enumerate(flat_keypoints):
            row[f'kp_{i}'] = val
        dataset_rows.append(row)

        print(f"✅ Procesado: {fname}")

# Guardar CSV final dentro de la carpeta CSV creada
csv_path = os.path.join(output_csv_folder, 'pose_keypoints_dataset.csv')
df = pd.DataFrame(dataset_rows)
df.to_csv(csv_path, index=False)
print(f"✅ Dataset CSV guardado en {csv_path}.")
