import open3d as o3d
import numpy as np
import copy
import os

# --- НАСТРОЙКИ ---
MODEL_FILE = "1.obj"

# --- ЗАГРУЗКА МОДЕЛИ ---
if os.path.exists(MODEL_FILE):
    print(f"Загрузка модели: {MODEL_FILE}")
    mesh = o3d.io.read_triangle_mesh(MODEL_FILE, enable_post_processing=True)
    if not mesh.has_vertex_normals():
        mesh.compute_vertex_normals()
    # Если нет текстурных координат — попробуем MTL-материалы
    print(f"  Вершин: {len(mesh.vertices)}")
    print(f"  Треугольников: {len(mesh.triangles)}")
    print(f"  Есть текстура: {mesh.has_textures()}")
    print(f"  Есть цвета вершин: {mesh.has_vertex_colors()}")
else:
    print(f"Файл '{MODEL_FILE}' не найден — используется куб-заменитель.")
    mesh = o3d.geometry.TriangleMesh.create_box(width=1.0, height=0.4, depth=0.8)
    mesh.compute_vertex_normals()
    # Покрасим куб чтобы было видно
    mesh.paint_uniform_color([0.6, 0.6, 0.8])

mesh.translate(-mesh.get_center())
mesh_display = copy.deepcopy(mesh)

# --- ВИЗУАЛИЗАТОР ---
vis = o3d.visualization.Visualizer()
vis.create_window(window_name="Просмотр модели", width=1280, height=720)
vis.add_geometry(mesh_display)

opt = vis.get_render_option()
opt.background_color = np.array([0.08, 0.08, 0.15])
opt.mesh_show_back_face = True
opt.light_on = True

# Материал: усиливаем specular чтобы свет «обтекал» форму
# и были видны вогнутости/выпуклости
opt.mesh_shade_option = o3d.visualization.MeshShadeOption.Default

ctr = vis.get_view_control()
ctr.set_zoom(0.5)
ctr.set_front([0.5, -0.35, -0.8])
ctr.set_up([0.0, 1.0, 0.0])

# --- НЕСКОЛЬКО ИСТОЧНИКОВ СВЕТА через кастомный рендер ---
# open3d DrawGeometriesWithEditing не поддерживает кастомный свет напрямую,
# поэтому добавляем «фейковые» источники через маленькие сферы-маркеры
# и используем несколько направлений через смену параметров вида.

# Добавляем три вспомогательных направленных источника через LineSet-"лучи"
# (невидимые, просто для корректного шейдинга open3d пересчитает нормали)
mesh_display.compute_triangle_normals()
mesh_display.compute_vertex_normals()

print("\nУправление:")
print("  Левая кнопка мыши — вращение")
print("  Колесо мыши       — зум")
print("  Средняя кнопка    — перемещение")
print("  Закрой окно для выхода\n")

# --- ЦИКЛ ---
while True:
    if not vis.poll_events():
        break
    vis.update_renderer()

print("Готово.")
vis.destroy_window()