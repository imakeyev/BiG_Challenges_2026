# -*- coding: utf-8 -*-
# Версия Blender: 4.5.0
# Описание: Финальная версия симулятора. Реализована настраиваемая система
# наведения сенсоров (на объект, в центр сцены или на середину траектории).

import bpy
import random
import math
from mathutils import Vector, Quaternion

# --- ГЛАВНЫЙ БЛОК НАСТРОЕК ---

# 1. Настройки объекта
OBJ_FILE_PATH = "1.obj"  # <--- ЗАМЕНИТЕ ЭТОТ ПУТЬ
USE_SIMPLE_SPIN_MODEL = True
OBJECT_COLOR_RGB = (0.5, 0.5, 0.5)
OBJECT_METALLIC = 0.8
MIN_ROTATION_SPEED = 0.01
MAX_ROTATION_SPEED = 0.04
MIN_LINEAR_SPEED = 0.01
MAX_LINEAR_SPEED = 0.05
MAX_DISTANCE_FROM_CENTER = 50.0

# 2. Настройки системы сенсоров ("Датчики")
STEREO_BASELINE = 0.5
LIDAR_OFFSET = Vector((0.0, 0.2, 0.0))
MIN_SENSOR_DISTANCE = 5.0
MAX_SENSOR_DISTANCE = 15.0

# 3. НОВЫЙ БЛОК: Настройка наведения сенсоров
# Выберите режим:
# 'OBJECT'   - Сенсоры динамически следят за летящим объектом.
# 'ORIGIN'   - Сенсоры смотрят строго в центр сцены (0, 0, 0).
# 'MIDPOINT' - Сенсоры смотрят в середину будущей траектории объекта (ИДЕАЛЬНЫЙ ВАРИАНТ).
TARGETING_MODE = 'MIDPOINT'

# --- КОНЕЦ НАСТРОЕК ---


# --- Глобальные переменные для движения (рассчитываются один раз) ---
ROTATION_AXIS = Vector((random.uniform(-1, 1), random.uniform(-1, 1), random.uniform(-1, 1))).normalized()
ROTATION_SPEED = random.uniform(MIN_ROTATION_SPEED, MAX_ROTATION_SPEED)
TUMBLING_VECTOR = Vector((
    random.uniform(MIN_ROTATION_SPEED, MAX_ROTATION_SPEED) * random.choice([-1, 1]),
    random.uniform(MIN_ROTATION_SPEED, MAX_ROTATION_SPEED) * random.choice([-1, 1]),
    random.uniform(MIN_ROTATION_SPEED, MAX_ROTATION_SPEED) * random.choice([-1, 1])
))
LINEAR_DIRECTION = Vector((random.uniform(-1, 1), random.uniform(-1, 1), random.uniform(-1, 1))).normalized()
LINEAR_SPEED_SCALAR = random.uniform(MIN_LINEAR_SPEED, MAX_LINEAR_SPEED)
LINEAR_VELOCITY_VECTOR = LINEAR_DIRECTION * LINEAR_SPEED_SCALAR


# --- Функции создания объектов и материалов (без изменений) ---
def clear_scene():
    bpy.ops.object.select_all(action='SELECT'); bpy.ops.object.delete()
    print("Сцена очищена.")
def setup_world():
    world = bpy.context.scene.world
    if world:
        if not world.node_tree: world.use_nodes = True
        bg_node = world.node_tree.nodes.get('Background')
        if bg_node: bg_node.inputs['Color'].default_value = (0,0,0,1)
    print("Фон мира настроен (космос).")
def add_sun():
    bpy.ops.object.light_add(type='SUN', location=(0,0,0)); sun=bpy.context.active_object
    sun.name="SunLight"; sun.rotation_euler=(random.uniform(0,6.28),random.uniform(0,6.28),random.uniform(0,6.28))
    sun.data.energy=random.uniform(5.0,10.0); print(f"Источник света 'Sun' добавлен.")
    return sun
def import_and_setup_object(filepath):
    try:bpy.ops.wm.obj_import(filepath=filepath)
    except Exception as e: print(f"ОШИБКА импорта: {e}"); return None
    obj=bpy.context.selected_objects[0]; obj.name="SpaceDebris"
    bpy.ops.object.origin_set(type='ORIGIN_GEOMETRY', center='BOUNDS'); obj.location=(0,0,0)
    print(f"Объект '{obj.name}' импортирован.")
    return obj
def create_custom_material(obj, color_rgb, metallic_value):
    mat=bpy.data.materials.new(name="CustomMaterial"); mat.use_nodes=True
    nodes=mat.node_tree.nodes; nodes.clear()
    output_node=nodes.new(type='ShaderNodeOutputMaterial'); principled_bsdf=nodes.new(type='ShaderNodeBsdfPrincipled')
    noise_texture=nodes.new(type='ShaderNodeTexNoise'); color_ramp=nodes.new(type='ShaderNodeValToRGB')
    principled_bsdf.inputs['Base Color'].default_value=(*color_rgb, 1.0)
    principled_bsdf.inputs['Metallic'].default_value=metallic_value
    noise_texture.inputs['Scale'].default_value=random.uniform(5.0,15.0)
    color_ramp.color_ramp.elements[0].position=0.4; color_ramp.color_ramp.elements[1].position=0.8
    links=mat.node_tree.links
    links.new(principled_bsdf.outputs['BSDF'], output_node.inputs['Surface'])
    links.new(noise_texture.outputs['Fac'], color_ramp.inputs['Fac'])
    links.new(color_ramp.outputs['Color'], principled_bsdf.inputs['Roughness'])
    if obj.data.materials: obj.data.materials[0]=mat
    else: obj.data.materials.append(mat)
    print(f"Материал создан.")


# --- ОБНОВЛЕННАЯ ФУНКЦИЯ СОЗДАНИЯ СЕНСОРОВ ---
def create_sensor_rig():
    """
    Создает физическую систему сенсоров ("Датчики") без логики наведения.
    """
    bpy.ops.object.empty_add(type='PLAIN_AXES', location=(0,0,0))
    sensor_rig_parent = bpy.context.active_object
    sensor_rig_parent.name = "SensorRig"
    
    bpy.ops.object.camera_add(location=(-STEREO_BASELINE / 2, 0, 0))
    cam_left = bpy.context.active_object
    cam_left.name = "Camera.L"; cam_left.parent = sensor_rig_parent
    
    bpy.ops.object.camera_add(location=(STEREO_BASELINE / 2, 0, 0))
    cam_right = bpy.context.active_object
    cam_right.name = "Camera.R"; cam_right.parent = sensor_rig_parent
    
    bpy.ops.object.empty_add(type='CUBE', location=LIDAR_OFFSET)
    lidar = bpy.context.active_object
    lidar.name = "Lidar"; lidar.parent = sensor_rig_parent
    lidar.scale = (0.2, 0.2, 0.2)
    
    distance = random.uniform(MIN_SENSOR_DISTANCE, MAX_SENSOR_DISTANCE)
    random_direction = Vector((random.uniform(-1, 1), random.uniform(-1, 1), random.uniform(-1, 1))).normalized()
    sensor_rig_parent.location = random_direction * distance
    
    print(f"Система сенсоров 'SensorRig' создана на расстоянии {distance:.2f} м.")
    return sensor_rig_parent

# --- Логика анимации (без изменений) ---
def update_object_transform_handler(scene):
    obj = scene.objects.get("SpaceDebris")
    if not obj: return
    if obj.location.length > MAX_DISTANCE_FROM_CENTER:
        if 'animation_stopped' not in scene or not scene['animation_stopped']:
            print(f"Объект достиг макс. расстояния. Анимация остановлена.")
            bpy.ops.screen.animation_cancel(restore_frame=False); scene['animation_stopped'] = True
        return
    if USE_SIMPLE_SPIN_MODEL:
        obj.rotation_quaternion @= Quaternion(ROTATION_AXIS, ROTATION_SPEED)
    else:
        obj.rotation_euler.x += TUMBLING_VECTOR.x; obj.rotation_euler.y += TUMBLING_VECTOR.y; obj.rotation_euler.z += TUMBLING_VECTOR.z
    obj.location += LINEAR_VELOCITY_VECTOR
def register_animation_handler():
    old_handlers = [h for h in bpy.app.handlers.frame_change_pre if h.__name__ == 'update_object_transform_handler']
    for h in old_handlers: bpy.app.handlers.frame_change_pre.remove(h)
    bpy.app.handlers.frame_change_pre.append(update_object_transform_handler)
    print("Обработчик анимации зарегистрирован.")


# --- ОБНОВЛЕННАЯ ТОЧКА ВХОДА В СКРИПТ ---
if __name__ == "__main__":
    clear_scene()
    setup_world()
    add_sun()
    space_debris = import_and_setup_object(OBJ_FILE_PATH)
    sensor_rig = create_sensor_rig()
    
    if space_debris and sensor_rig:
        # --- НОВАЯ ЛОГИКА НАВЕДЕНИЯ ---
        target_point = None
        if TARGETING_MODE == 'OBJECT':
            target_point = space_debris
            print("Режим наведения: слежение за объектом.")
        
        elif TARGETING_MODE == 'ORIGIN' or TARGETING_MODE == 'MIDPOINT':
            bpy.ops.object.empty_add(type='SPHERE', radius=0.1)
            target_point = bpy.context.active_object
            target_point.name = "TargetPoint"
            
            if TARGETING_MODE == 'ORIGIN':
                target_point.location = (0, 0, 0)
                print("Режим наведения: взгляд в центр сцены (0,0,0).")
            else: # MIDPOINT
                midpoint_location = LINEAR_DIRECTION * (MAX_DISTANCE_FROM_CENTER / 2.0)
                target_point.location = midpoint_location
                print(f"Режим наведения: взгляд на середину траектории {midpoint_location.to_tuple(2)}.")
        
        if target_point:
            constraint = sensor_rig.constraints.new(type='TRACK_TO')
            constraint.target = target_point
            constraint.track_axis = 'TRACK_NEGATIVE_Z'
            constraint.up_axis = 'UP_Y'
        else:
            print("ОШИБКА: Неверный режим наведения. Сенсоры не нацелены.")

        # --- Остальная логика ---
        if USE_SIMPLE_SPIN_MODEL: space_debris.rotation_mode = 'QUATERNION'
        else: space_debris.rotation_mode = 'XYZ'
        
        create_custom_material(space_debris, OBJECT_COLOR_RGB, OBJECT_METALLIC)
        register_animation_handler()
        
        bpy.context.scene['animation_stopped'] = False
        bpy.context.scene.frame_end = 1000
        bpy.context.scene.frame_current = 1
        
        print("\n--- СЦЕНА ГОТОВА! ---")
        print("Нажмите ПРОБЕЛ в окне 3D Viewport для запуска анимации.")
    else:
        print("\n--- ОШИБКА ---")
        print("Скрипт не был выполнен до конца.")