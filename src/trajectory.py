# point_trajectory.py
import open3d as o3d
import numpy as np
from scipy.spatial.transform import Rotation
import time
import os

# --- 1. НАСТРОЙКИ (меняй здесь, чтобы подобрать идеальную траекторию) ---
MODEL_FILE = "1.obj"
NUM_STEPS = 800
TIME_STEP = 0.05
velocity = np.array([0.1, 0.3, 0.1])
acceleration = np.array([0.0, -0.01, 0.0])
angular_velocity = np.array([1.2, 0.8, -0.5])

# --- 2. ПОДГОТОВКА СЦЕНЫ И ВЫБОР ТОЧКИ ---
if os.path.exists(MODEL_FILE):
    print(f"Загрузка модели из файла: {MODEL_FILE}")
    mesh = o3d.io.read_triangle_mesh(MODEL_FILE)
else:
    print(f"Файл '{MODEL_FILE}' не найден. Создание объекта-заменителя (куб).")
    mesh = o3d.geometry.TriangleMesh.create_box(width=1.0, height=0.4, depth=0.8)

mesh.translate(-mesh.get_center())
vertices = np.asarray(mesh.vertices)
farthest_vertex_index = np.argmax(vertices[:, 1])
tracked_point_local = vertices[farthest_vertex_index]
print(f"Выбрана точка для отслеживания: {tracked_point_local}")

trajectory_line = o3d.geometry.LineSet()
trajectory_points = []

# --- 3. ИНИЦИАЛИЗАЦИЯ ВИЗУАЛИЗАТОРА ---
vis = o3d.visualization.Visualizer()
vis.create_window(window_name="Траектория точки на вращающемся объекте", width=1280, height=720)

vis.add_geometry(trajectory_line)
opt = vis.get_render_option()
opt.background_color = np.asarray([0.0, 0.0, 0.0])
ctr = vis.get_view_control()
ctr.set_zoom(0.4)

# Начальные условия
current_position = np.array([0.0, 0.0, 0.0])
total_rotation_matrix = np.identity(3)

# --- 4. ЦИКЛ АНИМАЦИИ ---
print("Запуск анимации... Нажмите Ctrl+C в терминале или закройте окно для выхода.")

# --- ИСПРАВЛЕНИЕ ЗДЕСЬ ---
# Делаем один проход рендера ПЕРЕД циклом, чтобы окно инициализировалось.
vis.poll_events()
vis.update_renderer()
# --- КОНЕЦ ИСПРАВЛЕНИЯ ---

try:
    for i in range(NUM_STEPS):
        velocity += acceleration * TIME_STEP
        current_position += velocity * TIME_STEP
        
        delta_rotation = Rotation.from_rotvec(angular_velocity * TIME_STEP)
        total_rotation_matrix = delta_rotation.as_matrix() @ total_rotation_matrix

        rotated_point_vec = total_rotation_matrix @ tracked_point_local
        tracked_point_world = current_position + rotated_point_vec
        
        trajectory_points.append(tracked_point_world)
        
        if len(trajectory_points) > 1:
            lines = [[j, j + 1] for j in range(len(trajectory_points) - 1)]
            colors = [[c, 1 - c, 0.2] for c in np.linspace(1, 0.2, len(lines