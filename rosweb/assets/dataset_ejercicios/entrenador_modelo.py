import pandas as pd
import numpy as np
from sklearn.model_selection import train_test_split
from sklearn.ensemble import GradientBoostingClassifier
from sklearn.preprocessing import LabelEncoder
from sklearn.metrics import classification_report, confusion_matrix
import joblib
import os

# === CONFIGURACIÓN ===
csv_path = 'csv_output/pose_keypoints_dataset.csv'
modelo_path = 'modelo_fases_pose.pkl'
label_encoder_path = 'label_encoder.pkl'
ideal_pose_path = 'pose_promedios.pkl'

# === FUNCIÓN: EXTRAER FEATURES SEMÁNTICOS ===
def extraer_features(df_kp):
    num_keypoints = df_kp.shape[1] // 2
    coords = df_kp.values.reshape((-1, num_keypoints, 2))

    features = []

    for pose in coords:
        pose_feats = []

        # Distancia hombros (kp_5, kp_6)
        d_hombros = np.linalg.norm(pose[5] - pose[6])
        pose_feats.append(d_hombros)

        # Distancia caderas (kp_11, kp_12)
        d_caderas = np.linalg.norm(pose[11] - pose[12])
        pose_feats.append(d_caderas)

        # Ángulo entre hombros y caderas
        v1 = pose[5] - pose[11]
        v2 = pose[6] - pose[12]
        cos_ang = np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2) + 1e-6)
        angle = np.arccos(np.clip(cos_ang, -1.0, 1.0))
        pose_feats.append(angle)

        # Altura relativa de las manos (kp_9, kp_10) vs caderas (kp_11, kp_12)
        mano_izq_arriba = pose[9][1] < pose[11][1]
        mano_der_arriba = pose[10][1] < pose[12][1]
        pose_feats.append(int(mano_izq_arriba))
        pose_feats.append(int(mano_der_arriba))

        # Simetría izquierda-derecha (piernas, brazos)
        sim_brazos = np.linalg.norm(pose[5] - pose[7]) - np.linalg.norm(pose[6] - pose[8])
        sim_piernas = np.linalg.norm(pose[11] - pose[13]) - np.linalg.norm(pose[12] - pose[14])
        pose_feats.append(sim_brazos)
        pose_feats.append(sim_piernas)

        features.append(pose_feats)

    return np.array(features)

# === 1. CARGAR EL CSV ===
df = pd.read_csv(csv_path)
keypoints_cols = [col for col in df.columns if col.startswith('kp_')]
assert len(keypoints_cols) == 34, f"❌ El CSV tiene {len(keypoints_cols)} columnas de keypoints, se esperaban 34"

# === 2. EXTRAER FEATURES ===
X = extraer_features(df[keypoints_cols])
y = df['label']

# === 3. CODIFICAR ETIQUETAS ===
le = LabelEncoder()
y_encoded = le.fit_transform(y)

# === 4. DIVIDIR DATOS ===
X_train, X_test, y_train, y_test = train_test_split(X, y_encoded, test_size=0.2, random_state=42)

# === 5. ENTRENAR MODELO (mejor que RF) ===
clf = GradientBoostingClassifier(random_state=42)
clf.fit(X_train, y_train)

# === 6. EVALUAR ===
y_pred = clf.predict(X_test)
print("📊 Classification Report:")
print(classification_report(y_test, y_pred, target_names=le.classes_))
print("🧾 Confusion Matrix:")
print(confusion_matrix(y_test, y_pred))

# === 7. GUARDAR MODELO Y ENCODER ===
joblib.dump(clf, modelo_path)
joblib.dump(le, label_encoder_path)
print(f"✅ Modelo guardado en {modelo_path}")
print(f"✅ Codificador guardado en {label_encoder_path}")

# === 8. GUARDAR POSES PROMEDIO (de los keypoints originales) ===
ideal_poses = df.groupby('label')[keypoints_cols].mean()
joblib.dump(ideal_poses, ideal_pose_path)
print(f"✅ Poses promedio por fase guardadas en {ideal_pose_path}")
