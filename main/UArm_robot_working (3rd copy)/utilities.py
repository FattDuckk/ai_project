import pybullet as p
import numpy as np

class Camera:
    def __init__(self, cam_pos, cam_tar, cam_up_vector, near=0.01, far=10.0, size=(320, 320), fov=60):
        self.width, self.height = size
        self.near, self.far = near, far
        self.fov = fov
        self.cam_pos = cam_pos
        self.cam_tar = cam_tar
        self.cam_up_vector = cam_up_vector
        self.view_matrix = None
        self.projection_matrix = None

    def update_view_matrix(self, cam_pos, cam_tar, cam_up_vector):
        self.cam_pos = cam_pos
        self.cam_tar = cam_tar
        self.cam_up_vector = cam_up_vector

        self.view_matrix = p.computeViewMatrix(cam_pos, cam_tar, cam_up_vector)
        self.projection_matrix = p.computeProjectionMatrixFOV(
            self.fov, self.width / self.height, self.near, self.far
        )
    def capture_image(self):
        images = p.getCameraImage(self.width, self.height, self.view_matrix, self.projection_matrix)
        rgb = np.reshape(images[2], (self.height, self.width, 4))[:, :, :3]
        depth = np.reshape(images[3], (self.height, self.width))
        return rgb, depth
