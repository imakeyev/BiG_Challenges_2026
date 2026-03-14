import open3d as o3d
import numpy as np
import os
import copy

# --- НАСТРОЙКИ ---
DATA_PATH = os.path.join("..", "data", "set_10") # Укажи свою папку с данными
START_FRAME = 1
END_FRAME = 200 # Обработаем побольше кадров для хорошей модели

def load_lidar_pcd(filepath):
    """Загружает облако точек лидара и сразу вычисляет нормали (нужны для ICP и мешинга)"""
    if not os.path.exists(filepath) or os.path.getsize(filepath) == 0: 
        return None
    points = np.loadtxt(filepath, dtype=np.float64)
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    
    # Вычисляем нормали (важно, чтобы они смотрели "наружу" от центра)
    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.5, max_nn=30))
    pcd.orient_normals_towards_camera_location(camera_location=np.array([0., 0., 0.]))
    
    return pcd

if __name__ == '__main__':
    print("=== ЭТАП 1: Сборка единого облака точек из сканов Лидара ===")
    
    global_pcd = o3d.geometry.PointCloud()
    current_pose = np.identity(4)
    pcd_prev = None

    for frame_id in range(START_FRAME, END_FRAME + 1):
        filepath = os.path.join(DATA_PATH, f"{frame_id:04d}", "lidar.xyz")
        pcd_current = load_lidar_pcd(filepath)
        
        if pcd_current is None or not pcd_current.has_points():
            continue
            
        print(f"\rОбработка кадра {frame_id}/{END_FRAME}", end="")
            
        if pcd_prev is not None:
            # Используем Point-to-Plane ICP для точного вычисления вращения объекта
            icp_result = o3d.pipelines.registration.registration_icp(
                pcd_prev, pcd_current, max_correspondence_distance=0.5,
                init=np.identity(4),
                estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPlane()
            )
            # Накапливаем позу объекта
            current_pose = icp_result.transformation @ current_pose
            
        # Берем текущий кадр, "отменяем" его вращение/полет и добавляем в общую копилку
        pcd_to_add = copy.deepcopy(pcd_current)
        pcd_to_add.transform(np.linalg.inv(current_pose))
        global_pcd += pcd_to_add
        
        pcd_prev = pcd_current

    print(f"\nСборка завершена. Всего собрано точек: {len(global_pcd.points)}")
    
    # Немного прореживаем облако для ускорения и сглаживания шумов
    print("Очистка и фильтрация облака...")
    global_pcd = global_pcd.voxel_down_sample(voxel_size=0.05)
    # Удаляем статистические выбросы (случайный мусор)
    global_pcd, ind = global_pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
    
    # Еще раз пересчитываем нормали для итогового чистого облака
    global_pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.2, max_nn=30))
    global_pcd.orient_normals_consistent_tangent_plane(100)


    print("\n=== ЭТАП 2: Реконструкция 3D-модели (Мешинг Пуассона) ===")
    print("Натягиваем полигональную сетку на точки...")
    
    # Алгоритм Poisson Surface Reconstruction
    # depth=9 означает хорошую детализацию. Если компьютер слабый, поставь 8.
    mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(global_pcd, depth=9)
    
    # Пуассон иногда создает лишнюю "пленку" вокруг модели. 
    # Отрезаем полигоны, в которых плотность точек была слишком низкой.
    print("Удаление артефактов...")
    vertices_to_remove = densities < np.quantile(densities, 0.05)
    mesh.remove_vertices_by_mask(vertices_to_remove)
    
    # Раскрасим модель в приятный серый цвет, как настоящий 3D-объект
    mesh.paint_uniform_color([0.7, 0.7, 0.7])
    mesh.compute_vertex_normals() # Чтобы красиво отражался свет при рендере

    print("\n=== ГОТОВО ===")
    
    # Сохраняем результат в файл
    output_mesh_path = os.path.join(DATA_PATH, "reconstructed_lidar_mesh.ply")
    o3d.io.write_triangle_mesh(output_mesh_path, mesh)
    print(f"3D модель сохранена в: {output_mesh_path}")

    # Показываем результат
    print("Открываем окно визуализации (Светло-серая модель - готовый МЕШ)")
    
    # Создаем оси координат для понимания пространства
    coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=2.0)
    
    o3d.visualization.draw_geometries([mesh, coord_frame], window_name="Итоговый 3D-меш (только LiDAR)", mesh_show_wireframe=False, mesh_show_back_face=True)
