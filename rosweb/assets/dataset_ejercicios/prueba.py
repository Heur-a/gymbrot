import numpy as np
import joblib

# === INPUT: Keypoints normalizados (ejemplo formato lista) ===
keypoints_flat = [0.6065503358840942, 0.43527016043663025, 0.6181297898292542, 0.42434120178222656, 0.6241350769996643, 0.42377594113349915, 0.6306623220443726, 0.4232698082923889, 0.6070504188537598, 0.42607519030570984, 0.6042764186859131, 0.42655545473098755, 0.6016441583633423, 0.42688751220703125, 0.6529293060302734, 0.42957836389541626, 0.6138938069343567, 0.43099382519721985, 0.6215674877166748, 0.44868290424346924, 0.6061927676200867, 0.4480929970741272, 0.6628453731536865, 0.48617926239967346, 0.612350344657898, 0.481032133102417, 0.6384708881378174, 0.5558169484138489, 0.5732784271240234, 0.5351488590240479, 0.5629057288169861, 0.5937015414237976, 0.5135558247566223, 0.5592733025550842]
# === Cargar modelo y encoder ===
modelo_path = 'modelo_fases_pose.pkl'
label_encoder_path = 'label_encoder.pkl'

clf = joblib.load(modelo_path)
le = joblib.load(label_encoder_path)

# === Feature extraction (igual que en entrenamiento) ===
def extraer_features_unicos(flat_keypoints):
    coords = np.array(flat_keypoints).reshape((17, 2))
    feats = []

    d_hombros = np.linalg.norm(coords[5] - coords[6])
    feats.append(d_hombros)

    d_caderas = np.linalg.norm(coords[11] - coords[12])
    feats.append(d_caderas)

    v1 = coords[5] - coords[11]
    v2 = coords[6] - coords[12]
    cos_ang = np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2) + 1e-6)
    angle = np.arccos(np.clip(cos_ang, -1.0, 1.0))
    feats.append(angle)

    mano_izq_arriba = coords[9][1] < coords[11][1]
    mano_der_arriba = coords[10][1] < coords[12][1]
    feats.append(int(mano_izq_arriba))
    feats.append(int(mano_der_arriba))

    sim_brazos = np.linalg.norm(coords[5] - coords[7]) - np.linalg.norm(coords[6] - coords[8])
    sim_piernas = np.linalg.norm(coords[11] - coords[13]) - np.linalg.norm(coords[12] - coords[14])
    feats.append(sim_brazos)
    feats.append(sim_piernas)

    return np.array(feats).reshape(1, -1)

# === Predecir ===
X_input = extraer_features_unicos(keypoints_flat)
pred = clf.predict(X_input)[0]
fase = le.inverse_transform([pred])[0]

print(f"📌 Fase estimada: {fase}")
