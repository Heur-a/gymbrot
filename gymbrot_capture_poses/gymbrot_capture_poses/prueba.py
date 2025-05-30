import cv2

video_path = '/home/marcos04/turtlebot3_ws/install/gymbrot_capture_poses/share/gymbrot_capture_poses/resources/video_referencia.mp4'
cap = cv2.VideoCapture(video_path)

if not cap.isOpened():
    print("❌ No se pudo abrir el video")
else:
    print("✅ Video abierto correctamente")
    ret, frame = cap.read()
    if ret:
        print("✅ Primer frame leído correctamente")
    else:
        print("❌ No se pudo leer el primer frame")
cap.release()
