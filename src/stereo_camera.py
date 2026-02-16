import cv2
import numpy as np
import json
from preprocessor import preprocess_stereo_pair # Импортируем наш препроцессор

class StereoCamera:
    def __init__(self, config_path="config.json"):
        with open(config_path, 'r') as f:
            config = json.load(f)

        cam_conf = config['camera']
        
        # Загружаем параметры камеры
        self.width = cam_conf['width']
        self.height = cam_conf['height']
        self.K1 = self._get_cam_matrix(cam_conf['intrinsics'])
        self.D1 = np.array(cam_conf['distortion_coeffs'])
        
        # Для стереопары нам нужна вторая матрица. 
        # В симуляции и для простоты предположим, что они идентичны.
        self.K2 = self.K1
        self.D2 = self.D1

        # Параметры стерео, включая baseline
        baseline = cam_conf['baseline_m']
        # Матрица вращения R и вектор смещения T между камерами
        # В нашей идеальной установке нет вращения (R=единичная) и смещение только по X.
        self.R = np.eye(3)
        self.T = np.array([-baseline, 0, 0]) # Смещение по оси X

        # --- Предварительные вычисления для ректификации ---
        self.R1, self.R2, self.P1, self.P2, self.Q, _, _ = cv2.stereoRectify(
            self.K1, self.D1, self.K2, self.D2, 
            (self.width, self.height), 
            self.R, self.T, alpha=0
        )
        self.map1_l, self.map2_l = cv2.initUndistortRectifyMap(self.K1, self.D1, self.R1, self.P1, (self.width, self.height), cv2.CV_32FC1)
        self.map1_r, self.map2_r = cv2.initUndistortRectifyMap(self.K2, self.D2, self.R2, self.P2, (self.width, self.height), cv2.CV_32FC1)

        # --- Настройка алгоритма StereoSGBM ---
        # Это базовые параметры, их можно и нужно тюнить
        num_disp = 128
        block_size = 5
        self.stereo = cv2.StereoSGBM_create(
            minDisparity=0,
            numDisparities=num_disp,
            blockSize=block_size,
            P1=8 * 3 * block_size**2,
            P2=32 * 3 * block_size**2,
            disp12MaxDiff=1,
            uniquenessRatio=10,
            speckleWindowSize=100,
            speckleRange=32
        )

        print("StereoCamera module initialized.")

    def _get_cam_matrix(self, intrinsics):
        """Собирает матрицу камеры из параметров."""
        fx, fy, cx, cy = intrinsics['fx'], intrinsics['fy'], intrinsics['cx'], intrinsics['cy']
        return np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])

    def get_point_cloud(self, left_img, right_img):
        """
        Главный метод: принимает стереопару и возвращает облако точек.
        """
        # 1. Ректификация (выравнивание) изображений
        left_rect = cv2.remap(left_img, self.map1_l, self.map2_l, cv2.INTER_LINEAR)
        right_rect = cv2.remap(right_img, self.map1_r, self.map2_r, cv2.INTER_LINEAR)
        
        # 2. Предобработка: убираем блики и тени
        left_processed, right_processed = preprocess_stereo_pair(left_rect, right_rect)

        # 3. Вычисление карты глубины
        disparity_map = self.stereo.compute(left_processed, right_processed).astype(np.float32) / 16.0

        # 4. Преобразование в 3D облако точек
        points_3D = cv2.reprojectImageTo3D(disparity_map, self.Q)
        
        # Фильтруем некорректные точки (где disparity был <= 0)
        mask = disparity_map > disparity_map.min()
        points_3D = points_3D[mask]
        
        # Также отфильтруем точки, которые слишком далеко
        points_3D = points_3D[points_3D[:, 2] < 100] # Оставляем точки ближе 100 метров

        return points_3D

