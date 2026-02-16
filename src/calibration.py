import cv2
import numpy as np
import glob
import json
import os
import sys
import time

class StereoCalibrator:
    IMAGES_PATH = os.path.join("data", "calib_images")
    OUTPUT_FILE = "camera_calib.json"
    CHECKERBOARD_DIMS = (9, 6)
    SQUARE_SIZE_M = 0.025

    def __init__(self):
        os.makedirs(self.IMAGES_PATH, exist_ok=True)

    def create_dataset(self, num_images=15, cam_l_idx=0, cam_r_idx=1):
        print(f"\n--- Create calibration dataset ---")
        print(f"Required images: {num_images}. Press [SPACE] to capture, [Q] to quit.")
        
        cap_l = cv2.VideoCapture(cam_l_idx)
        cap_r = cv2.VideoCapture(cam_r_idx)
        
        if not cap_l.isOpened() or not cap_r.isOpened():
            print(f"[ERROR] Could not open cameras ({cam_l_idx}, {cam_r_idx}).")
            return

        img_counter = 0
        while img_counter < num_images:
            ret_l, frame_l = cap_l.read()
            ret_r, frame_r = cap_r.read()
            if not ret_l or not ret_r:
                print("[ERROR] Failed to get frame.")
                break
            
            # Display both streams side-by-side
            combined_frame = np.hstack((frame_l, frame_r))
            cv2.putText(combined_frame, f"Capture {img_counter+1}/{num_images}", 
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.imshow("Stereo Capture", combined_frame)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord(' '):
                fname_l = os.path.join(self.IMAGES_PATH, f"left_{img_counter:02d}.png")
                fname_r = os.path.join(self.IMAGES_PATH, f"right_{img_counter:02d}.png")
                cv2.imwrite(fname_l, frame_l)
                cv2.imwrite(fname_r, frame_r)
                print(f"Saved pair {img_counter+1}: {os.path.basename(fname_l)}, {os.path.basename(fname_r)}")
                img_counter += 1
                
                # Show a "captured" message briefly
                cv2.putText(combined_frame, "CAPTURED!", (200, 250), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 255), 3)
                cv2.imshow("Stereo Capture", combined_frame)
                cv2.waitKey(500) # 0.5 second pause

        cap_l.release()
        cap_r.release()
        cv2.destroyAllWindows()
        print(f"\n--- Dataset creation finished. ---")

    def run_calibration(self):
        print("\n--- Running stereo calibration ---")
        objp = np.zeros((self.CHECKERBOARD_DIMS[0] * self.CHECKERBOARD_DIMS[1], 3), np.float32)
        objp[:, :2] = np.mgrid[0:self.CHECKERBOARD_DIMS[0], 0:self.CHECKERBOARD_DIMS[1]].T.reshape(-1, 2)
        objp *= self.SQUARE_SIZE_M

        objpoints, imgpoints_l, imgpoints_r = [], [], []
        images_l = sorted(glob.glob(os.path.join(self.IMAGES_PATH, 'left_*.png')))
        images_r = sorted(glob.glob(os.path.join(self.IMAGES_PATH, 'right_*.png')))

        if not images_l or len(images_l) != len(images_r):
            print(f"[ERROR] No valid image pairs found in '{self.IMAGES_PATH}'.")
            return False

        print(f"Found {len(images_l)} image pairs for analysis.")
        img_shape = None

        for fname_l, fname_r in zip(images_l, images_r):
            img_l = cv2.imread(fname_l)
            gray_l = cv2.cvtColor(img_l, cv2.COLOR_BGR2GRAY)
            if img_shape is None: img_shape = gray_l.shape[::-1]

            img_r = cv2.imread(fname_r)
            gray_r = cv2.cvtColor(img_r, cv2.COLOR_BGR2GRAY)

            ret_l, corners_l = cv2.findChessboardCorners(gray_l, self.CHECKERBOARD_DIMS, None)
            ret_r, corners_r = cv2.findChessboardCorners(gray_r, self.CHECKERBOARD_DIMS, None)

            if ret_l and ret_r:
                objpoints.append(objp)
                criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
                imgpoints_l.append(cv2.cornerSubPix(gray_l, corners_l, (11, 11), (-1, -1), criteria))
                imgpoints_r.append(cv2.cornerSubPix(gray_r, corners_r, (11, 11), (-1, -1), criteria))
        
        if not objpoints:
            print("[ERROR] Checkerboard not found in any image pair. Calibration failed.")
            return False
            
        print("Performing stereo calibration...")
        ret, mtx_l, dist_l, mtx_r, dist_r, R, T, E, F = cv2.stereoCalibrate(
            objpoints, imgpoints_l, imgpoints_r, None, None, None, None, img_shape
        )

        if not ret:
            print("[ERROR] Stereo calibration failed.")
            return False

        self._save_results(mtx_l, dist_l, np.linalg.norm(T), img_shape)
        return True

    def _save_results(self, mtx, dist, baseline, shape):
        data = {
            "stereo_baseline_m": baseline,
            "camera_parameters": {
                "width": shape[0], "height": shape[1],
                "intrinsics": {
                    "fx": mtx[0, 0], "fy": mtx[1, 1],
                    "cx": mtx[0, 2], "cy": mtx[1, 2]
                },
                "distortion": dist.flatten().tolist()
            }
        }
        with open(self.OUTPUT_FILE, 'w') as f:
            json.dump(data, f, indent=4)
        print(f"[OK] Calibration results saved to '{self.OUTPUT_FILE}'")

if __name__ == "__main__":
    calibrator = StereoCalibrator()
    
    # Проверяем, есть ли уже изображения
    image_files = glob.glob(os.path.join(calibrator.IMAGES_PATH, '*.png'))
    
    if not image_files:
        # Если изображений нет, запускаем процесс создания
        print("No calibration images found.")
        calibrator.create_dataset()
    
    # После создания (или если они уже были), запускаем калибровку
    if not calibrator.run_calibration():
        sys.exit(1) # Выход с кодом ошибки, если калибровка не удалась
    
    sys.exit(0) # Успешный выход
