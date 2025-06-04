import cv2
import numpy as np
from estima_pose import PoseEstimator
import estima_pose
print("Estima_pose.py importado desde:", estima_pose.__file__)
import importlib
importlib.reload(estima_pose)

cv2.startWindowThread()
cv2.namedWindow("Video - Estimación de Pose", cv2.WINDOW_NORMAL)

video_path = 'video_referencia.mp4'
estimator = estima_pose.PoseEstimator()

cap = cv2.VideoCapture(video_path)
if not cap.isOpened():
    print(f"[ERROR] No se pudo abrir el video: {video_path}")
    exit(1)

frame_idx = 0

print("Presiona 's' para guardar la pose del frame actual.")
print("Presiona 'q' para salir.")

while cap.isOpened():
    ret, frame = cap.read()
    ret, frame = cap.read()
    print(f"[DEBUG] ret = {ret}, frame shape = {frame.shape if frame is not None else 'None'}")

    if not ret:
        print("Fin del video o error.")
        break

    # Mostrar frame
    import inspect
    print(inspect.signature(estimator.estimate))
    keypoints, _, vis_img = estimator.estimate(frame, return_vis=True)
    display = vis_img if vis_img is not None else frame
    cv2.imshow("Video - Estimación de Pose", display)

    key = cv2.waitKey(30) & 0xFF
    if key == ord('s'):
        if keypoints is not None:
            np.save('pose_referencia.npy', keypoints)
            cv2.imwrite('frame_referencia.jpg', display)
            print(f"Pose guardada en frame {frame_idx}.")
        else:
            print("No se detectó una pose clara en este frame.")
    elif key == ord('q'):
        print("Saliendo...")
        break

    frame_idx += 1

cap.release()
cv2.destroyAllWindows()
