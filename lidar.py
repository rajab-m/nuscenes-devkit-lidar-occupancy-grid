
from nuscenes.nuscenes import NuScenes
from nuscenes.utils.data_classes import LidarPointCloud
from nuscenes.utils.geometry_utils import transform_matrix
from pyquaternion import Quaternion

import numpy as np



# ---------------------------------------------------------
# LiDAR processing
# ---------------------------------------------------------
class Lidar:
    def __init__(self, nusc):
        self.nusc = nusc

    def get_sample_token(self, scene_number, sample_nbr):
        """Return the token of a sample in a scene."""

        if scene_number < 0 or scene_number >= len(self.nusc.scene):
            raise IndexError("Invalid scene number.")

        scene = self.nusc.scene[scene_number]
        samples_num = scene['nbr_samples']

        if sample_nbr < 0 or sample_nbr >= samples_num:
            raise IndexError(f"sample_nbr must be between 0 and {samples_num - 1}")

        token = scene['first_sample_token']

        for _ in range(sample_nbr):
            sample = self.nusc.get('sample', token)
            token = sample['next']

        return token

    # -----------------------------------------------------
    def get_lidar_cloud(self,scene_number, sample_number):
        """Load and filter LiDAR points in the LiDAR frame."""

        sample_token = self.get_sample_token(scene_number, sample_number)
        sample = self.nusc.get('sample', sample_token)

        lidar_token = sample['data']['LIDAR_TOP']
        file_path = self.nusc.get_sample_data_path(lidar_token)

        cloud = LidarPointCloud.from_file(file_path)
        points = cloud.points

        x = points[0, :]
        y = points[1, :]
        z = points[2, :]

        # remove ground points, basic filtering
        mask = ((z > -1.2) &((np.abs(x) > 1.5) | (np.abs(y) > 2.5)))

        return x[mask], y[mask], z[mask]

    # -----------------------------------------------------
    def get_transforms(self,scene_number, sample_number):
        """Return LiDAR-to-ego and ego-to-world transformations."""

        sample_token = self.get_sample_token(scene_number, sample_number)
        sample = self.nusc.get('sample', sample_token)

        lidar_token = sample['data']['LIDAR_TOP']
        sample_data = self.nusc.get('sample_data', lidar_token)

        calibrated_sensor = self.nusc.get('calibrated_sensor',sample_data['calibrated_sensor_token'])

        ego_pose = self.nusc.get('ego_pose',sample_data['ego_pose_token'])

        T_ego_lidar = transform_matrix(calibrated_sensor['translation'],Quaternion(calibrated_sensor['rotation']))

        T_world_ego = transform_matrix(ego_pose['translation'],Quaternion(ego_pose['rotation']))

        return T_ego_lidar, T_world_ego

    # -----------------------------------------------------
    def get_lidar_sample(self,scene_number, sample_number):
        """Return filtered LiDAR x and y in world coordinates."""

        x, y, z = self.get_lidar_cloud(scene_number, sample_number)

        # Homogeneous point matrix: 4 x N
        points_lidar = np.vstack((x,y,z,np.ones(x.shape[0])))

        T_ego_lidar, T_world_ego = self.get_transforms(scene_number, sample_number)

        T_world_lidar = T_world_ego @ T_ego_lidar

        points_world = T_world_lidar @ points_lidar

        # world x and y
        x_world = points_world[0, :]
        y_world = points_world[1, :]

        return x_world, y_world

    # -----------------------------------------------------
    def get_lidar_pose(self,scene_number, sample_number):
        """Return the LiDAR sensor's world position as [x, y]."""

        T_ego_lidar, T_world_ego = self.get_transforms(scene_number, sample_number)

        T_world_lidar = T_world_ego @ T_ego_lidar

        lidar_origin = np.array([0.0, 0.0, 0.0, 1.0])
        world_position = T_world_lidar @ lidar_origin

        return world_position[:2]

    # -----------------------------------------------------
    def get_all_points(self,scene_number):
        """Collect world-coordinate XY points and their sensor poses."""

        samples_num = self.nusc.scene[scene_number]['nbr_samples']

        all_points = []
        all_poses = []

        for sample_number in range(samples_num):

            x, y = self.get_lidar_sample(scene_number, sample_number)

            pose = self.get_lidar_pose(scene_number, sample_number)

            # Each row represents one world-coordinate XY point
            sample_points = np.column_stack((x, y))
            all_points.append(sample_points)

            # Associate the sensor pose with each point
            sample_poses = np.tile(pose, (sample_points.shape[0], 1))
            all_poses.append(sample_poses)

        if not all_points:
            return np.empty((0, 2)), np.empty((0, 2))

        all_points = np.concatenate(all_points, axis=0)
        all_poses = np.concatenate(all_poses, axis=0)

        return all_points, all_poses

