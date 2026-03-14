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

# --- "ПЕСОЧНИЦА" ДЛЯ НАСТРОЙКИ ---
# 1. Масштаб
SCALE_CAMERA = 5 # Попробуй это значение, оно должно быть очень близко

# 2. Дополнительное вращение (в градусах)
# Эти "ручки" для тонкой подстройки
ROT_X = 0.0
ROT_Y = 20.0 # Основной разворот на 180 уже включен
ROT_Z = 180.0
# -----------------------------------

if __name__ == '__main__':
    DATA_PATH = "..\data\set_08" 
    FRAME_ID = 5
    CONFIG_FILE = "..\config.json"
    frame_folder = os.path.join(DATA_PATH, f"{FRAME_ID:04d}")
    
    stereo_cam = StereoCamera(config_path=CONFIG_FILE)
    
    left_image = cv2.imread(os.path.join(frame_folder, "left.png"))
    right_image = cv2.imread(os.path.join(frame_folder, "right.png"))
    
    if left_image is not None and right_image is not None:
        pcd_camera_local = o3d.geometry.PointCloud()
        points_local, colors_local = stereo_cam.get_point_cloud(left_image, right_image)
        if points_local.shape[0] > 0:
            pcd_camera_local.points = o3d.utility.Vector3dVector(points_local)
            pcd_camera_local.colors = o3d.utility.Vector3dVector(colors_local / 255.0)
        
        pcd_lidar = load_lidar_pcd(os.path.join(frame_folder, "lidar.xyz"))
        
        if not pcd_camera_local.has_points() or not pcd_lidar.has_points():
            print("ОШИБКА: Одно из облаков точек пустое.")
        else:
            # --- ПРАВИЛЬНАЯ ЛОГИКА ТРАНСФОРМАЦИИ ---
            
            # 1. Находим центры исходных облаков
            center_lidar_world = pcd_lidar.get_center()
            center_camera_local = pcd_camera_local.get_center()
            
            # 2. Создаем трансформацию для центрирования облака камеры
            T_center_cam = np.identity(4)
            T_center_cam[:3, 3] = -center_camera_local
            
            # 3. Создаем трансформацию для масштабирования
            T_scale = np.identity(4)
            T_scale[0,0] = T_scale[1,1] = T_scale[2,2] = SCALE_CAMERA
            
            # 4. Создаем трансформацию для вращения
            # Сначала обязательный разворот на 180 градусов по Y
            R_180 = pcd_camera_local.get_rotation_matrix_from_xyz((0, np.pi, 0))
            # Потом дополнительное вращение из "ручек"
            R_manual = pcd_camera_local.get_rotation_matrix_from_xyz((
                np.deg2rad(ROT_X), np.deg2rad(ROT_Y), np.deg2rad(ROT_Z)
            ))
            # Объединяем вращения
            T_rotate = np.identity(4)
            T_rotate[:3, :3] = R_manual @ R_180

            # 5. Создаем трансформацию для возвращения на место лидара
            T_return_to_lidar = np.identity(4)
            T_return_to_lidar[:3, 3] = center_lidar_world

            # --- Собираем ВСЕ трансформации в одну финальную матрицу ---
            # Порядок важен: сначала центрируем, потом масштабируем, потом вращаем, потом возвращаем
            T_final = T_return_to_lidar @ T_rotate @ T_scale @ T_center_cam
            
            # --- Трансформируем облако камеры ОДИН РАЗ ---
            pcd_camera_transformed = pcd_camera_local.transform(T_final)
            
            # --- ВИЗУАЛИЗАЦИЯ ---
            pcd_lidar.paint_uniform_color([0, 0.65, 0.9])
            coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0)
            
            print("Отображение: Синий - Лидар, Цветное - Камера (полная трансформация)")
            o3d.visualization.draw_geometries(
                [pcd_camera_transformed], 
                window_name="Песочница v3.0"
            )

