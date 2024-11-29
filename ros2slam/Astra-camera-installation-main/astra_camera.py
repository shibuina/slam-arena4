import cv2
import numpy as np
from openni import openni2
from openni import _openni2 as c_api
import open3d as o3d  # Dùng để xử lý Point Cloud


class Camera:
    def __init__(self, fps=30, width=640, height=480, openni_libs='/home/sora/slam-arena4/ros2slam/Astra-camera-installation-main/libs'):
        self.fps = fps
        self.width = width
        self.height = height
        self.openni_libs = openni_libs
        self.wait_time = int(1000.0 / float(fps))
        self.load()

    def unload(self):
        openni2.unload()

    def load(self):
        try:
            openni2.initialize(self.openni_libs)
            self.dev = openni2.Device.open_any()

            # Initialize depth stream
            self.depth_stream = self.dev.create_depth_stream()
            self.depth_stream.start()
            self.depth_stream.set_video_mode(
                c_api.OniVideoMode(
                    pixelFormat=c_api.OniPixelFormat.ONI_PIXEL_FORMAT_DEPTH_100_UM,
                    resolutionX=self.width,
                    resolutionY=self.height,
                    fps=self.fps,
                )
            )
        except Exception as e:
            print(f"Error loading OpenNI: {e}")
            exit()

    def get_depth(self):
        frame = self.depth_stream.read_frame()
        frame_data = frame.get_buffer_as_uint16()

        if len(frame_data) != self.height * self.width * 2:  # 2 bytes per uint16
            print("Invalid depth frame size!")
            return None

        depth_frame = np.frombuffer(frame_data, dtype=np.uint16).reshape(self.height, self.width)
        return depth_frame


def detect_planes(depth_frame):
    """
    Phát hiện mặt phẳng (tường và mặt đất) từ Depth Frame.
    """
    points = []
    for y in range(depth_frame.shape[0]):
        for x in range(depth_frame.shape[1]):
            z = depth_frame[y, x]
            if z > 0:  # Loại bỏ điểm không hợp lệ
                points.append([x, y, z])

    # Chuyển dữ liệu thành Point Cloud
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np.array(points))

    # RANSAC để phát hiện mặt phẳng
    plane_model, inliers = pcd.segment_plane(distance_threshold=0.02,
                                             ransac_n=3,
                                             num_iterations=1000)
    [a, b, c, d] = plane_model

    # Lọc các điểm thuộc mặt phẳng và các điểm còn lại
    ground_cloud = pcd.select_by_index(inliers)
    wall_cloud = pcd.select_by_index(inliers, invert=True)

    return ground_cloud, wall_cloud, plane_model


def visualize_planes(ground_cloud, wall_cloud):
    """
    Hiển thị mặt đất và tường bằng Open3D.
    """
    ground_cloud.paint_uniform_color([1, 0, 0])  # Mặt đất màu đỏ
    wall_cloud.paint_uniform_color([0, 0, 1])  # Tường màu xanh dương

    o3d.visualization.draw_geometries([ground_cloud, wall_cloud])


def main():
    # Khởi tạo camera
    camera = Camera()

    while True:
        # Lấy Depth Frame
        depth_frame = camera.get_depth()
        if depth_frame is None:
            print("Failed to retrieve depth frame. Skipping...")
            continue

        # Hiển thị Depth Frame chuẩn hóa
        normalized_depth = cv2.normalize(depth_frame, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
        cv2.imshow("Depth Frame", normalized_depth)

        # Nhấn 'd' để phát hiện tường và mặt đất
        key = cv2.waitKey(1) & 0xFF
        if key == ord('d'):
            ground_cloud, wall_cloud, plane_model = detect_planes(depth_frame)
            print(f"Detected Plane: {plane_model}")
            visualize_planes(ground_cloud, wall_cloud)

        # Nhấn 'q' để thoát
        if key == ord('q'):
            break

    # Dọn dẹp tài nguyên
    camera.unload()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
