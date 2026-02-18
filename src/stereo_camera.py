import cv2
import numpy as np
import json
# preprocessor.py должен быть в той же папке
from preprocessor import create_bad_pixel_mask

class StereoCamera:
    def __init__(self, config_path="config.json"):
        with open(config_path, 'r') as f:
            config = json.load(f)

        cam_conf = config['camera']
        
        self.width = cam_conf['width']
        self.height = cam_conf['height']
        self.K1 = self._get_cam_matrix(cam_conf['intrinsics'])
        self.D1 = np.array(cam_conf['distortion_coeffs'])
        self.K2 = self.K1; self.D2 = self.D1

        baseline = cam_conf['baseline_m']
        self.R = np.eye(3); self.T = np.array([-baseline, 0, 0])

        self.R1, self.R2, self.P1, self.P2, self.Q, _, _ = cv2.stereoRectify(
            self.K1, self.D1, self.K2, self.D2, 
            (self.width, self.height), self.R, self.T, alpha=0
        )
        self.map1_l, self.map2_l = cv2.initUndistortRectifyMap(self.K1, self.D1, self.R1, self.P1, (self.width, self.height), cv2.CV_32FC1)
        self.map1_r, self.map2_r = cv2.initUndistortRectifyMap(self.K2, self.D2, self.R2, self.P2, (self.width, self.height), cv2.CV_32FC1)

        num_disp = 128; block_size = 5
        self.stereo = cv2.StereoSGBM_create(
            minDisparity=0, numDisparities=num_disp, blockSize=block_size, P1=8*3*block_size**2, 
            P2=32*3*block_size**2, disp12MaxDiff=1, uniquenessRatio=15,
            speckleWindowSize=100, speckleRange=32, mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY
        )
        print("StereoCamera module initialized.")

    def _get_cam_matrix(self, intrinsics):
        fx, fy, cx, cy = intrinsics['fx'], intrinsics['fy'], intrinsics['cx'], intrinsics['cy']
        return np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])

# Внутри класса StereoCamera в файле stereo_camera.py

    def get_point_cloud(self, left_img, right_img):
        left_rect = cv2.remap(left_img, self.map1_l, self.map2_l, cv2.INTER_LINEAR)
        right_rect = cv2.remap(right_img, self.map1_r, self.map2_r, cv2.INTER_LINEAR)
        
        left_gray = cv2.cvtColor(left_rect, cv2.COLOR_BGR2GRAY)
        right_gray = cv2.cvtColor(right_rect, cv2.COLOR_BGR2GRAY)
        
        disparity_map = self.stereo.compute(left_gray, right_gray).astype(np.float32) / 16.0
        points_3D = cv2.reprojectImageTo3D(disparity_map, self.Q)
        
        colors = cv2.cvtColor(left_rect, cv2.COLOR_BGR2RGB)
        
        # --- МАСКА ДЛЯ ВСЕГО "ПЛОХОГО" ---
        
        # 1. Невалидные значения disparity (где SGBM ничего не нашел)
        mask_disparity = disparity_map > self.stereo.getMinDisparity()
        
        # 2. Черные пиксели на изображении (где нет текстуры)
        SHADOW_THRESHOLD = 15 # Можно сделать 20 или 25, если нужно еще агрессивнее
        mask_shadow = left_gray > SHADOW_THRESHOLD
        
        # 3. Бесконечные или невалидные 3D-точки
        # reprojectImageTo3D создает точки с Z=10000 для невалидных disparity
        mask_inf = points_3D[:, :, 2] < 1000 
        
        # Объединяем все маски
        final_mask = mask_disparity & mask_shadow & mask_inf
        
        # Применяем маску
        points_3D = points_3D[final_mask]
        colors = colors[final_mask]
        
        return points_3D, colors
