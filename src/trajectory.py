import open3d as o3d
import numpy as np
from scipy.spatial.transform import Rotation
import time
import copy
import os

# --- 1. НАСТРОЙКИ ---
MODEL_FILE = "1.obj"
NUM_STEPS = 500
TIME_STEP = 0.05

# Кривая траектория: ненулевые X и Z в ускорении дают изгиб во всех осях
velocity         = np.array([0.15,  0.40,  0.05])
acceleration     = np.array([0.02, -0.015, 0.008])
angular_velocity = np.array([0.8, -0.5, 0.3])

# Длина отрезка, показывающего ось вращения в каждый момент
AXIS_TAIL_LENGTH = 0.6

# --- 2. ЗАГРУЗКА МОДЕЛИ ---
if os.path.exists(MODEL_FILE):
    print(f"Загрузка модели: {MODEL_FILE}")
    mesh = o3d.io.read_triangle_mesh(MODEL_FILE)
    if not mesh.has_vertex_normals():
        mesh.compute_vertex_normals()
else:
    print(f"Файл '{MODEL_FILE}' не найден — используется куб-заменитель.")
    mesh = o3d.geometry.TriangleMesh.create_box(width=1.0, height=0.4, depth=0.8)
    mesh.compute_vertex_normals()

mesh.translate(-mesh.get_center())
original_vertices = np.asarray(mesh.vertices).copy()
mesh_display = copy.deepcopy(mesh)

# --- 3. ЛИНИИ ---
traj_line = o3d.geometry.LineSet()   # траектория центра масс
axis_line = o3d.geometry.LineSet()   # след оси вращения

# --- 4. ВИЗУАЛИЗАТОР ---
vis = o3d.visualization.Visualizer()
vis.create_window(window_name="Траектория спутника", width=1280, height=720)
vis.add_geometry(mesh_display)
vis.add_geometry(traj_line)
vis.add_geometry(axis_line)

opt = vis.get_render_option()
opt.line_width = 6.0                              # толщина линий
opt.background_color = np.array([0.05, 0.05, 0.12])

ctr = vis.get_view_control()
ctr.set_zoom(0.35)
ctr.set_front([0.4, -0.25, -0.88])
ctr.set_up([0.0, 1.0, 0.0])

# --- 5. СОСТОЯНИЕ ---
current_position      = np.array([0.0, 0.0, 0.0])
vel                   = velocity.copy()
total_rotation_matrix = np.eye(3)

traj_points = []
axis_points = []

print("Анимация запущена. Закройте окно для выхода.")


def set_lineset(ls, points, lines, colors):
    ls.points = o3d.utility.Vector3dVector(points)
    ls.lines  = o3d.utility.Vector2iVector(lines)
    ls.colors = o3d.utility.Vector3dVector(colors)


# --- 6. ЦИКЛ ---
for i in range(NUM_STEPS):

    # Физика
    vel += acceleration * TIME_STEP
    current_position += vel * TIME_STEP

    # Вращение
    delta_rot = Rotation.from_rotvec(angular_velocity * TIME_STEP)
    total_rotation_matrix = delta_rot.as_matrix() @ total_rotation_matrix

    # Обновление модели
    rotated    = (total_rotation_matrix @ original_vertices.T).T
    translated = rotated + current_position
    mesh_display.vertices = o3d.utility.Vector3dVector(translated)
    mesh_display.compute_vertex_normals()
    vis.update_geometry(mesh_display)

    # --- Линия 1: траектория центра масс ---
    traj_points.append(current_position.copy())

    if len(traj_points) > 1:
        n = len(traj_points)
        lines = [[j, j + 1] for j in range(n - 1)]
        # Оранжевый → голубой
        colors = []
        for j in range(n - 1):
            t = j / max(n - 2, 1)
            colors.append([1.0 - t * 0.85, 0.45 + t * 0.35, 0.1 + t * 0.9])
        set_lineset(traj_line, traj_points, lines, colors)
        vis.update_geometry(traj_line)

    # --- Линия 2: мгновенная ось вращения ---
    # Локальная ось Z объекта (третий столбец матрицы вращения)
    local_z    = total_rotation_matrix[:, 2]
    axis_start = current_position - local_z * AXIS_TAIL_LENGTH * 0.5
    axis_end   = current_position + local_z * AXIS_TAIL_LENGTH * 0.5

    axis_points.append(axis_start.copy())
    axis_points.append(axis_end.copy())

    if len(axis_points) >= 4:
        num_segs = len(axis_points) // 2
        lines_ax = [[j * 2, j * 2 + 1] for j in range(num_segs)]
        # Зелёный → жёлтый
        colors_ax = []
        for j in range(num_segs):
            t = j / max(num_segs - 1, 1)
            colors_ax.append([0.2 + t * 0.8, 0.9 - t * 0.3, 0.1])
        set_lineset(axis_line, axis_points, lines_ax, colors_ax)
        vis.update_geometry(axis_line)

    if not vis.poll_events():
        print("Окно закрыто.")
        break
    vis.update_renderer()
    time.sleep(0.01)

    if i % 50 == 0:
        p = current_position
        print(f"  Шаг {i:4d}/{NUM_STEPS}  pos=({p[0]:.2f}, {p[1]:.2f}, {p[2]:.2f})")

print("Готово.")
vis.destroy_window()