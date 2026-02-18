import json
import numpy as np
from scipy.spatial.transform import Rotation
import subprocess
import sys
import os
import argparse

class ConfigGenerator:
    CAMERA_CALIB_FILE = "camera_calib.json"
    CONFIG_FILE = "config.json"
    
    def __init__(self, simulation_mode, camera_args):
        self.is_sim = simulation_mode
        self.camera_args = camera_args
        print(f"Mode: {'simulation' if self.is_sim else 'real'}")

    def run(self):
        if self.is_sim:
            self._generate_for_simulation()
        else:
            self._generate_for_real()
        self._assemble_and_save_config()

    def _generate_for_simulation(self):
        self.cam_params = {
            "width": self.camera_args['width'], "height": self.camera_args['height'],
            "intrinsics": {
                "fx": self.camera_args['fx'], "fy": self.camera_args['fy'],
                "cx": self.camera_args['cx'], "cy": self.camera_args['cy']
            },
            "distortion": [0.0, 0.0, 0.0, 0.0, 0.0]
        }
        self.baseline = 0.7 # Синхронизировано с генератором
        self.lidar_pos = [0.0, 0.2, 0.0]
        self.lidar_rot = Rotation.from_euler('xyz', [0, 0, 0], degrees=True)

    def _generate_for_real(self):
        # ... (эта часть без изменений)
        if not os.path.exists(self.CAMERA_CALIB_FILE):
            print(f"Calibration file not found. Running calibration.py...")
            self._run_calibration_module()
        if not os.path.exists(self.CAMERA_CALIB_FILE):
            print(f"[ERROR] Calibration module did not create file. Exiting.")
            sys.exit(1)
        with open(self.CAMERA_CALIB_FILE, 'r') as f:
            calib_data = json.load(f)
        self.cam_params = calib_data['camera_parameters']
        self.baseline = calib_data['stereo_baseline_m']
        print("Note: Using predefined lidar position for real hardware.")
        self.lidar_pos = [0.0, 0.05, 0.02]
        self.lidar_rot = Rotation.from_euler('xyz', [0, 0, 0], degrees=True)

    def _run_calibration_module(self):
        try:
            script_path = os.path.join("src", "calibration.py")
            subprocess.run([sys.executable, script_path], check=True)
        except (subprocess.CalledProcessError, FileNotFoundError):
            print(f"[ERROR] Calibration module failed or not found.")
            sys.exit(1)

    def _assemble_and_save_config(self):
        rot_matrix = self.lidar_rot.as_matrix()
        transform = np.eye(4)
        transform[:3, :3] = rot_matrix
        transform[:3, 3] = self.lidar_pos
        
        # --- ВОТ ЭТОТ БЛОК ДОЛЖЕН БЫТЬ В ФАЙЛЕ ---
        manual_calibration = {
            "scale_camera": 5.0,
            "rot_x_deg": 0.0,
            "rot_y_deg": 20.0,
            "rot_z_deg": 180.0
        }
        # ---------------------------------------------

        config = {
            "camera": {
                "type": "Stereo", "width": self.cam_params['width'], "height": self.cam_params['height'],
                "intrinsics": self.cam_params['intrinsics'],
                "distortion_coeffs": self.cam_params.get('distortion', []),
                "baseline_m": self.baseline
            },
            "lidar": {
                "type": "VirtualLidar" if self.is_sim else "RealLidar",
                "extrinsics_lidar_to_camera": transform.tolist()
            },
            "manual_calibration_simulator": manual_calibration # <--- И ЭТА СЕКЦИЯ
        }
        
        with open(self.CONFIG_FILE, "w") as f:
            json.dump(config, f, indent=4)
        print(f"[OK] Configuration saved to '{self.CONFIG_FILE}'")

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Sensor configuration generator.")
    parser.add_argument('-m', '--mode', type=str, default='sim', choices=['sim', 'real'], help="Mode: 'sim' or 'real'")
    parser.add_argument('--width', type=int, default=1280, help='Image width')
    parser.add_argument('--height', type=int, default=720, help='Image height')
    parser.add_argument('--fx', type=float, default=800.0, help='Focal length x')
    parser.add_argument('--fy', type=float, default=800.0, help='Focal length y')
    parser.add_argument('--cx', type=float, default=640.0, help='Principal point x')
    parser.add_argument('--cy', type=float, default=360.0, help='Principal point y')
    args = parser.parse_args()
    camera_args_dict = {'width': args.width, 'height': args.height, 'fx': args.fx, 'fy': args.fy, 'cx': args.cx, 'cy': args.cy}
    generator = ConfigGenerator(simulation_mode=(args.mode == 'sim'), camera_args=camera_args_dict)
    generator.run()

