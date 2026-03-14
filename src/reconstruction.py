import open3d as o3d
import numpy as np
import json
import cv2
import os
import copy
from stereo_camera import StereoCamera

# ... (get_fused_pcd_for_frame и load_lidar_pcd без изменений)
def load_lidar_pcd(filepath):
    if not os.path.exists(filepath) or os.path.getsize(filepath) == 0: return o3d.geometry.PointCloud()
    points = np.loadtxt(filepath, dtype=np.float64); pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points); return pcd
def get_fused_pcd_for_frame(frame_folder, stereo_cam, calib_params):
    left_image = cv2.imread(os.path.join(frame_folder, "left.png")); right_image = cv2.imread(os.path.join(frame_folder, "right.png"))
    pcd_lidar = load_lidar_pcd(os.path.join(frame_folder, "lidar.xyz"))
    pcd_camera_local = o3d.geometry.PointCloud()
    if left_image is not None and right_image is not None:
        points_local, colors_local = stereo_cam.get_point_cloud(left_image, right_image)
        if points_local.shape[0] > 0: pcd_camera_local.points = o3d.utility.Vector3dVector(points_local)
    if not pcd_camera_local.has_points() or not pcd_lidar.has_points(): return None
    center_lidar_world = pcd_lidar.get_center(); center_camera_local = pcd_camera_local.get_center()
    T_center_cam = np.identity(4); T_center_cam[:3, 3] = -center_camera_local
    T_scale = np.identity(4); T_scale[0,0]=T_scale[1,1]=T_scale[2,2] = calib_params['scale']
    R_180 = pcd_camera_local.get_rotation_matrix_from_xyz((0, np.pi, 0))
    R_manual = pcd_camera_local.get_rotation_matrix_from_xyz((np.deg2rad(calib_params['rot_x']), np.deg2rad(calib_params['rot_y']), np.deg2rad(calib_params['rot_z'])))
    T_rotate = np.identity(4); T_rotate[:3, :3] = R_manual @ R_180
    T_return_to_lidar = np.identity(4); T_return_to_lidar[:3, 3] = center_lidar_world
    T_rough_align = T_return_to_lidar @ T_rotate @ T_scale @ T_center_cam
    pcd_camera_aligned_roughly = o3d.geometry.PointCloud(pcd_camera_local).transform(T_rough_align)
    threshold = 0.5
    icp_transform = o3d.pipelines.registration.registration_icp(pcd_camera_aligned_roughly, pcd_lidar, threshold, np.identity(4), o3d.pipelines.registration.TransformationEstimationPointToPoint()).transformation
    pcd_camera_aligned_finely = pcd_camera_aligned_roughly.transform(icp_transform)
    fused_pcd = pcd_camera_aligned_finely + pcd_lidar
    fused_pcd = fused_pcd.voxel_down_sample(voxel_size=0.1)
    return fused_pcd

if __name__ == '__main__':
    # --- НАСТРОЙКИ ---
    DATA_PATH = os.path.join("..", "data", "set_10")
    CONFIG_FILE = os.path.join("..", "config.json")
    START_FRAME = 1; END_FRAME = 200
    
    with open(CONFIG_FILE, 'r') as f: config = json.load(f)
    calib_params = { 'scale': config['manual_calibration_simulator']['scale_camera'], 'rot_x': config['manual_calibration_simulator']['rot_x_deg'], 'rot_y': config['manual_calibration_simulator']['rot_y_deg'], 'rot_z': config['manual_calibration_simulator']['rot_z_deg'] }
    stereo_cam = StereoCamera(config_path=CONFIG_FILE)
    
    world_pcd = o3d.geometry.PointCloud()
    current_pose = np.identity(4)
    pcd_prev = None

    print("--- НАЧАЛО 4D-РЕКОНСТРУКЦИИ (Point-to-Plane ICP) ---")
    
    for frame_id in range(START_FRAME, END_FRAME + 1):
        frame_folder = os.path.join(DATA_PATH, f"{frame_id:04d}")
        print(f"\n--- Обработка кадра {frame_id} ---")
        
        pcd_current = get_fused_pcd_for_frame(frame_folder, stereo_cam, calib_params)
        
        if pcd_current is None or not pcd_current.has_points():
            print("  Пропуск кадра."); continue
        
        # --- ИЗМЕНЕНИЕ: Вычисляем нормали для ICP ---
        pcd_current.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.3, max_nn=30))
        
        if pcd_prev is not None:
            # --- ИЗМЕНЕНИЕ: Используем Point-to-Plane ICP ---
            icp_result = o3d.pipelines.registration.registration_icp(
                pcd_prev, # Source
                pcd_current, # Target
                0.5, # Порог расстояния
                np.identity(4), # Начальная трансформация
                o3d.pipelines.registration.TransformationEstimationPointToPlane() # <-- ГЛАВНОЕ ИЗМЕНЕНИЕ
            )
            transformation_delta = icp_result.transformation
            current_pose = transformation_delta @ current_pose
        
        pcd_to_add = copy.deepcopy(pcd_current)
        pcd_to_add.transform(np.linalg.inv(current_pose))
        world_pcd += pcd_to_add
        print(f"  Всего в модели: {len(world_pcd.points)} точек.")

        pcd_prev = pcd_current

    print("\n--- РЕКОНСТРУКЦИЯ ЗАВЕРШЕНА ---")
    
    world_pcd = world_pcd.voxel_down_sample(voxel_size=0.05)
    print(f"Итоговая модель содержит {len(world_pcd.points)} точек.")
    
    o3d.visualization.draw_geometries([world_pcd], window_name="Итоговая 3D-реконструкция (Point-to-Plane)")
