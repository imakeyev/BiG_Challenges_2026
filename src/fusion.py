import open3d as o3d
import numpy as np
import json
import cv2
import os
from stereo_camera import StereoCamera

# --- Функции загрузки (без изменений) ---
def load_lidar_pcd(filepath):
    if not os.path.exists(filepath) or os.path.getsize(filepath) == 0: return o3d.geometry.PointCloud()
    points = np.loadtxt(filepath, dtype=np.float64); pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points); return pcd

# --- КОЭФФИЦИЕНТЫ ГРУБОЙ НАСТРОЙКИ (подобранные тобой) ---
# Эти значения используются для первоначального, "грубого" совмещения
MANUAL_SCALE = 1.0
MANUAL_ROT_X = 0.0
MANUAL_ROT_Y = 20.0
MANUAL_ROT_Z = 180.0
# ---------------------------------------------------------

if __name__ == '__main__':
    DATA_PATH = "..\data\set_08" # ВАЖНО: Будем генерировать в новую папку
    FRAME_ID = 20
    CONFIG_FILE = "..\config.json"
    frame_folder = os.path.join(DATA_PATH, f"{FRAME_ID:04d}")
    
    # 1. Загрузка данных
    stereo_cam = StereoCamera(config_path=CONFIG_FILE)
    left_image = cv2.imread(os.path.join(frame_folder, "left.png"))
    right_image = cv2.imread(os.path.join(frame_folder, "right.png"))
    pcd_lidar = load_lidar_pcd(os.path.join(frame_folder, "lidar.xyz"))
    
    pcd_camera_local = o3d.geometry.PointCloud()
    if left_image is not None and right_image is not None:
        points_local, colors_local = stereo_cam.get_point_cloud(left_image, right_image)
        if points_local.shape[0] > 0:
            pcd_camera_local.points = o3d.utility.Vector3dVector(points_local)
            pcd_camera_local.colors = o3d.utility.Vector3dVector(colors_local / 255.0)

    if not pcd_camera_local.has_points() or not pcd_lidar.has_points():
        print("ОШИБКА: Одно из облаков точек пустое.")
    else:
        # 2. Грубое совмещение по твоим коэффициентам
        # --- (логика из sandbox) ---
        center_lidar_world = pcd_lidar.get_center()
        center_camera_local = pcd_camera_local.get_center()
        T_center_cam = np.identity(4); T_center_cam[:3, 3] = -center_camera_local
        T_scale = np.identity(4); T_scale[0,0]=T_scale[1,1]=T_scale[2,2] = MANUAL_SCALE
        R_180 = pcd_camera_local.get_rotation_matrix_from_xyz((0, np.pi, 0))
        R_manual = pcd_camera_local.get_rotation_matrix_from_xyz((np.deg2rad(MANUAL_ROT_X), np.deg2rad(MANUAL_ROT_Y), np.deg2rad(MANUAL_ROT_Z)))
        T_rotate = np.identity(4); T_rotate[:3, :3] = R_manual @ R_180
        T_return_to_lidar = np.identity(4); T_return_to_lidar[:3, 3] = center_lidar_world
        T_rough_align = T_return_to_lidar @ T_rotate @ T_scale @ T_center_cam
        
        pcd_camera_aligned_roughly = o3d.geometry.PointCloud(pcd_camera_local).transform(T_rough_align)
        
        # 3. Точное совмещение с помощью ICP
        print("Выполняется точное совмещение с помощью ICP...")
        
        # Настройки ICP
        threshold = 0.5 # Максимальное расстояние между соответствующими точками
        # Исходная трансформация - единичная, т.к. мы уже сделали грубое совмещение
        trans_init = np.identity(4) 
        
        # Запускаем алгоритм
        reg_p2p = o3d.pipelines.registration.registration_icp(
            pcd_camera_aligned_roughly, # source (что двигаем)
            pcd_lidar,                  # target (к чему прицеливаемся)
            threshold, 
            trans_init,
            o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=2000)
        )
        
        # Получаем матрицу точной доводки
        icp_transform = reg_p2p.transformation
        print("Матрица доводки ICP:")
        print(icp_transform)
        
        # Применяем эту дополнительную трансформацию
        pcd_camera_aligned_finely = pcd_camera_aligned_roughly.transform(icp_transform)
        
        # 4. Создаем финальное объединенное облако
        fused_pcd = pcd_camera_aligned_finely + pcd_lidar
        fused_pcd_downsampled = fused_pcd.voxel_down_sample(voxel_size=0.1)
        print(f"Финальное облако: {len(fused_pcd_downsampled.points)} точек.")

        # 5. Визуализация
        #pcd_lidar.paint_uniform_color([0, 0.65, 0.9])
        # pcd_camera_aligned_roughly.paint_uniform_color([1, 0, 0]) # Красный - грубое совмещение
        pcd_camera_aligned_finely.paint_uniform_color([1, 0.7, 0]) # Оранжевый - точное совмещение
        
        o3d.visualization.draw_geometries(
            [fused_pcd_downsampled], # <-- Рисуем только одно, объединенное облако
            window_name="Итоговый совмещенный меш"
        )
