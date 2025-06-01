import os
import cv2
from PIL import Image
import pillow_heif

input_folder = os.path.expanduser('~/turtlebot3_ws/src/gymbrot/rosweb/assets/dataset_ejercicios/Muñeco_Medio')
output_folder = os.path.expanduser('~/turtlebot3_ws/src/gymbrot/rosweb/assets/dataset_ejercicios/medio')
prefix = 'mid'
ext = '.png'

image_files = [f for f in os.listdir(input_folder) if f.lower().endswith(('.jpg', '.jpeg', '.png', '.heic'))]
image_files.sort()

for idx, filename in enumerate(image_files, 1):
    path = os.path.join(input_folder, filename)
    new_name = f"{prefix}_{idx:03d}{ext}"
    output_path = os.path.join(output_folder, new_name)

    try:
        if filename.lower().endswith('.heic'):
            heif_file = pillow_heif.read_heif(path)
            image = heif_file.to_pillow()
            image.save(output_path, format='PNG')
            print(f"✅ HEIC convertido: {output_path}")
        else:
            img = cv2.imread(path)
            if img is None:
                print(f"❌ No se pudo leer: {filename}")
                continue
            cv2.imwrite(output_path, img)
            print(f"✅ Guardado: {output_path}")
    except Exception as e:
        print(f"❌ Error procesando {filename}: {e}")
