import bpy
import random
import math
import os
from mathutils import Vector, Quaternion

# --- БЛОК НАСТРОЕК ---
project_dir = os.path.dirname(bpy.data.filepath) if bpy.data.filepath else os.path.expanduser("~")
OBJ_FILE_PATH = os.path.join("1.obj")
OUTPUT_DATA_PATH = os.path.join("set_04") # Новая папка
TOTAL_FRAMES_TO_GENERATE = 100

# --- Параметры сцены и сенсоров ---
RESOLUTION_X, RESOLUTION_Y = 1280, 720
OBJECT_COLOR_RGB = (0.5, 0.5, 0.5); OBJECT_METALLIC = 0.8
STEREO_BASELINE = 0.5

# --- ИЗМЕНЕННЫЕ ПАРАМЕТРЫ ДВИЖЕНИЯ ---
# Увеличиваем скорости для более динамичной сцены
MIN_ROTATION_SPEED, MAX_ROTATION_SPEED = 0.05, 0.15 # Было 0.01, 0.04
LINEAR_SPEED_SCALAR = 0.3 # Было 0.2

# --- Глобальные переменные для движения ---
# Возвращаем беспорядочное вращение (Tumbling)
TUMBLING_VECTOR = Vector((
    random.uniform(MIN_ROTATION_SPEED, MAX_ROTATION_SPEED) * random.choice([-1, 1]),
    random.uniform(MIN_ROTATION_SPEED, MAX_ROTATION_SPEED) * random.choice([-1, 1]),
    random.uniform(MIN_ROTATION_SPEED, MAX_ROTATION_SPEED) * random.choice([-1, 1])
))
LINEAR_DIRECTION = Vector((random.uniform(-0.5, 0.5), random.uniform(-0.5, 0.5), -1.0)).normalized()
LINEAR_VELOCITY_VECTOR = LINEAR_DIRECTION * LINEAR_SPEED_SCALAR

# --- Функции создания сцены (свернуты) ---
def clear_scene():
    if bpy.context.object and bpy.context.object.mode == 'EDIT': bpy.ops.object.mode_set(mode='OBJECT')
    bpy.ops.object.select_all(action='SELECT'); bpy.ops.object.delete()
def setup_world():
    if bpy.context.scene.world:
        if not bpy.context.scene.world.node_tree: bpy.context.scene.world.use_nodes = True
        bpy.context.scene.world.node_tree.nodes.get('Background').inputs['Color'].default_value = (0,0,0,1)
def add_sun():
    bpy.ops.object.light_add(type='SUN', location=(0,0,0)); sun=bpy.context.active_object
    sun.name="SunLight"; sun.rotation_euler=(0.8, -0.5, 1.2); sun.data.energy=7.0
    return sun
def import_and_setup_object(filepath):
    try:
        bpy.ops.wm.obj_import(filepath=filepath)
        obj = bpy.context.selected_objects[0]
        obj.name = "SpaceDebris"
        bpy.ops.object.origin_set(type='ORIGIN_GEOMETRY', center='BOUNDS')
        obj.location = (0, 0, -15) # Начнем чуть дальше
        # --- ИЗМЕНЕНИЕ: Используем вращение Эйлера для Tumbling ---
        obj.rotation_mode = 'XYZ' 
        return obj
    except (IndexError, RuntimeError) as e:
        print(f"ОШИБКА: Не удалось импортировать или настроить объект: {e}")
        return None
def create_custom_material(obj, color_rgb, metallic_value):
    mat=bpy.data.materials.new(name="CustomMaterial"); mat.use_nodes=True; nodes=mat.node_tree.nodes; nodes.clear()
    output_node=nodes.new(type='ShaderNodeOutputMaterial'); principled_bsdf=nodes.new(type='ShaderNodeBsdfPrincipled');
    principled_bsdf.inputs['Base Color'].default_value=(*color_rgb, 1.0); principled_bsdf.inputs['Metallic'].default_value=metallic_value
    links=mat.node_tree.links; links.new(principled_bsdf.outputs['BSDF'], output_node.inputs['Surface'])
    if obj.data.materials: obj.data.materials[0]=mat
    else: obj.data.materials.append(mat)

# --- Точка входа в скрипт ---
if __name__ == "__main__":
    clear_scene(); setup_world(); add_sun()
    
    bpy.ops.object.camera_add(location=(-STEREO_BASELINE / 2, 0, 0)); cam_left = bpy.context.active_object; cam_left.name = "Camera.L"
    bpy.ops.object.camera_add(location=(STEREO_BASELINE / 2, 0, 0)); cam_right = bpy.context.active_object; cam_right.name = "Camera.R"
    
    space_debris = import_and_setup_object(OBJ_FILE_PATH)

    if space_debris:
        scene = bpy.context.scene
        scene.render.engine = 'CYCLES'; scene.cycles.device = 'CPU'; scene.cycles.samples = 16
        scene.render.resolution_x = RESOLUTION_X; scene.render.resolution_y = RESOLUTION_Y
        scene.render.image_settings.file_format = 'PNG'
        create_custom_material(space_debris, OBJECT_COLOR_RGB, OBJECT_METALLIC)
        
        print("\n--- СЦЕНА ГОТОВА! НАЧАЛО ГЕНЕРАЦИИ (СИЛЬНОЕ ВРАЩЕНИЕ)... ---")
        
        for frame_num in range(1, TOTAL_FRAMES_TO_GENERATE + 1):
            scene.frame_set(frame_num)
            
            if frame_num > 1:
                # --- ИЗМЕНЕНИЕ: Применяем Tumbling ---
                space_debris.rotation_euler.x += TUMBLING_VECTOR.x
                space_debris.rotation_euler.y += TUMBLING_VECTOR.y
                space_debris.rotation_euler.z += TUMBLING_VECTOR.z
                space_debris.location += LINEAR_VELOCITY_VECTOR
            
            print(f"--- Кадр {frame_num}: Объект в {space_debris.location.to_tuple(2)} ---")
            
            bpy.context.view_layer.update()
            depsgraph = bpy.context.evaluated_depsgraph_get()
            frame_folder = os.path.join(OUTPUT_DATA_PATH, f"{frame_num:04d}")
            os.makedirs(frame_folder, exist_ok=True)
            
            scene.camera = cam_left; scene.render.filepath = os.path.join(frame_folder, "left.png"); bpy.ops.render.render(write_still=True)
            scene.camera = cam_right; scene.render.filepath = os.path.join(frame_folder, "right.png"); bpy.ops.render.render(write_still=True)
        
        print("\n--- ГЕНЕРАЦИЯ ДАННЫХ УСПЕШНО ЗАВЕРШЕНА! ---")
    else:
        print("\n--- ОШИБКА: Не удалось создать сцену. ---")
