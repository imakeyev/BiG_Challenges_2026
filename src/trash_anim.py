import bpy
import random
import math
import os
import numpy as np # <-- ДОБАВЛЕН ИМПОРТ NUMPY
from mathutils import Vector, Quaternion

# --- БЛОК НАСТРОЕК ---
project_dir = os.path.dirname(bpy.data.filepath) if bpy.data.filepath else os.path.expanduser("~")
OBJ_FILE_PATH = os.path.join("1.obj")
OUTPUT_DATA_PATH = os.path.join("C:", "..", "data", "set_08")
TOTAL_FRAMES_TO_GENERATE = 50

# --- Параметры сцены и сенсоров ---
RESOLUTION_X, RESOLUTION_Y = 1280, 720
OBJECT_COLOR_RGB = (0.5, 0.5, 0.5); OBJECT_METALLIC = 0.8; STEREO_BASELINE = 0.3
MIN_ROTATION_SPEED, MAX_ROTATION_SPEED = 0.05, 0.15; LINEAR_SPEED_SCALAR = 0.3
LIDAR_SCAN_SIZE = 15.0; LIDAR_RESOLUTION = 256

# --- Глобальные переменные для движения ---
TUMBLING_VECTOR = Vector((random.uniform(MIN_ROTATION_SPEED, MAX_ROTATION_SPEED)*random.choice([-1, 1]), random.uniform(MIN_ROTATION_SPEED, MAX_ROTATION_SPEED)*random.choice([-1, 1]), random.uniform(MIN_ROTATION_SPEED, MAX_ROTATION_SPEED)*random.choice([-1, 1])))
LINEAR_DIRECTION = Vector((random.uniform(-0.5, 0.5), random.uniform(-0.5, 0.5), -1.0)).normalized()
LINEAR_VELOCITY_VECTOR = LINEAR_DIRECTION * LINEAR_SPEED_SCALAR

# --- Функции создания сцены ---
def clear_scene():
    if bpy.context.object and bpy.context.object.mode == 'EDIT': bpy.ops.object.mode_set(mode='OBJECT')
    bpy.ops.object.select_all(action='SELECT'); bpy.ops.object.delete()
def setup_world():
    if bpy.context.scene.world:
        if not bpy.context.scene.world.node_tree: bpy.context.scene.world.use_nodes = True
        bpy.context.scene.world.node_tree.nodes.get('Background').inputs['Color'].default_value = (0,0,0,1)
def add_sun():
    bpy.ops.object.light_add(type='SUN', location=(0,0,0)); sun=bpy.context.active_object
    sun.name="SunLight"; sun.rotation_euler=(0.8, -0.5, 1.2); sun.data.energy=10.0
    return sun
def import_and_setup_object(filepath):
    try:
        bpy.ops.wm.obj_import(filepath=filepath)
        obj = bpy.context.selected_objects[0]; obj.name = "SpaceDebris"
        max_dim = max(obj.dimensions); scale_factor = 5.0 / max_dim if max_dim > 0 else 1.0
        obj.scale = (scale_factor, scale_factor, scale_factor); bpy.ops.object.transform_apply(scale=True)
        bpy.ops.object.origin_set(type='ORIGIN_GEOMETRY', center='BOUNDS'); obj.location = (0, 0, -15)
        obj.rotation_mode = 'XYZ'; return obj
    except Exception as e: print(f"ОШИБКА: {e}"); return None
# --- ИЗМЕНЕННАЯ ФУНКЦИЯ СОЗДАНИЯ РЕАЛИСТИЧНОГО МАТЕРИАЛА ---
def create_realistic_cv_material(obj):
    mat = bpy.data.materials.new(name="Realistic_CV_Material")
    mat.use_nodes = True
    nodes = mat.node_tree.nodes
    nodes.clear()

    # --- Создаем узлы ---
    output = nodes.new(type='ShaderNodeOutputMaterial')
    principled = nodes.new(type='ShaderNodeBsdfPrincipled')
    tex_coord = nodes.new(type='ShaderNodeTexCoord')
    noise = nodes.new(type='ShaderNodeTexNoise')
    
    # --- Настраиваем узлы ---
    principled.inputs['Base Color'].default_value = (0.5, 0.5, 0.5, 1.0)
    principled.inputs['Metallic'].default_value = 0.9
    noise.inputs['Scale'].default_value = 150.0
    noise.inputs['Detail'].default_value = 16.0
    noise.inputs['Roughness'].default_value = 0.7
    
    # --- ИСПРАВЛЕНИЕ ЗДЕСЬ ---
    # Получаем доступ к 'links' через 'mat.node_tree'
    links = mat.node_tree.links
    
    # --- Соединяем узлы ---
    links.new(tex_coord.outputs['Generated'], noise.inputs['Vector'])
    links.new(noise.outputs['Fac'], principled.inputs['Roughness'])
    links.new(principled.outputs['BSDF'], output.inputs['Surface'])
    
    if obj.data.materials:
        obj.data.materials[0] = mat
    else:
        obj.data.materials.append(mat)


def grid_scan(depsgraph, lidar_obj, target_obj):
    points = []; lidar_origin = lidar_obj.matrix_world.to_translation(); target_center = target_obj.matrix_world.to_translation()
    direction = (target_center - lidar_origin).normalized()
    if direction.length < 0.001: direction = Vector((0,0,-1))
    if direction.z != 1.0 and direction.z != -1.0: right_axis = Vector((-direction.y, direction.x, 0)).normalized()
    else: right_axis = Vector((0, 1, 0))
    up_axis = direction.cross(right_axis); half_size = LIDAR_SCAN_SIZE / 2.0; step = LIDAR_SCAN_SIZE / (LIDAR_RESOLUTION - 1)
    for i in range(LIDAR_RESOLUTION):
        for j in range(LIDAR_RESOLUTION):
            x_offset = -half_size + j * step; y_offset = -half_size + i * step
            ray_origin = lidar_origin + right_axis * x_offset + up_axis * y_offset
            hit, location, _, _, _, _ = bpy.context.scene.ray_cast(depsgraph, ray_origin, direction, distance=100.0)
            if hit: points.append(location)
    return points

if __name__ == "__main__":
    clear_scene(); setup_world(); add_sun()
    
    bpy.ops.object.empty_add(type='PLAIN_AXES', location=(0,0,0)); sensor_rig = bpy.context.active_object; sensor_rig.name = "SensorRig"
    bpy.ops.object.camera_add(location=(-STEREO_BASELINE/2,0,0)); cam_left=bpy.context.active_object; cam_left.name="Camera.L"; cam_left.parent=sensor_rig
    bpy.ops.object.camera_add(location=(STEREO_BASELINE/2,0,0)); cam_right=bpy.context.active_object; cam_right.name="Camera.R"; cam_right.parent=sensor_rig
    
    space_debris = import_and_setup_object(OBJ_FILE_PATH)

    if space_debris:
        scene = bpy.context.scene; scene.render.engine = 'CYCLES'; scene.cycles.device = 'CPU'; scene.cycles.samples = 16
        scene.render.resolution_x = RESOLUTION_X; scene.render.resolution_y = RESOLUTION_Y; scene.render.image_settings.file_format = 'PNG'
        create_realistic_cv_material(space_debris)
        
        print("\n--- НАЧАЛО ГЕНЕРАЦИИ (с сохранением cam_pose.txt)... ---")
        
        for frame_num in range(1, TOTAL_FRAMES_TO_GENERATE + 1):
            scene.frame_set(frame_num)
            if frame_num > 1:
                space_debris.rotation_euler.x += TUMBLING_VECTOR.x; space_debris.rotation_euler.y += TUMBLING_VECTOR.y; space_debris.rotation_euler.z += TUMBLING_VECTOR.z
                space_debris.location += LINEAR_VELOCITY_VECTOR
            
            print(f"--- Кадр {frame_num} ---"); bpy.context.view_layer.update(); depsgraph = bpy.context.evaluated_depsgraph_get()
            frame_folder = os.path.join(OUTPUT_DATA_PATH, f"{frame_num:04d}"); os.makedirs(frame_folder, exist_ok=True)
            
            scene.camera = cam_left; scene.render.filepath = os.path.join(frame_folder, "left.png"); bpy.ops.render.render(write_still=True)
            scene.camera = cam_right; scene.render.filepath = os.path.join(frame_folder, "right.png"); bpy.ops.render.render(write_still=True)
            
            lidar_points = grid_scan(depsgraph, sensor_rig, space_debris); lidar_filepath = os.path.join(frame_folder, "lidar.xyz")
            with open(lidar_filepath, 'w') as f:
                for p in lidar_points: f.write(f"{p.x:.4f} {p.y:.4f} {p.z:.4f}\n")
            
            # --- ВОТ ЭТОТ БЛОК Я ЗАБЫЛ ВЕРНУТЬ ---
            cam_pose_matrix = cam_left.matrix_world
            pose_filepath = os.path.join(frame_folder, "cam_pose.txt")
            np.savetxt(pose_filepath, cam_pose_matrix)
            # ------------------------------------
        
        print("\n--- ГЕНЕРАЦИЯ ДАННЫХ УСПЕШНО ЗАВЕРШЕНА! ---")

