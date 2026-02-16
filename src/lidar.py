import json
import numpy as np

class Lidar:
    def __init__(self, config_path="config.json"):
        with open(config_path, 'r') as f:
            config = json.load(f)
        #матрицу обратно в массив
        self.extrinsics = np.array(config["lidar"]["extrinsics_lidar_to_camera"])

    def get_point_cloud(self, frame_id):
        #загрузка облака
        pass

if __name__ == '__main__':
    lidar_sensor = Lidar(config_path="../config.json")