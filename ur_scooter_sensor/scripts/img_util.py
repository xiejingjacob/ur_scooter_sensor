import numpy as np

def registerRGBToDepth(rgb_img, depth_img, rgb_info, depth_info, depth_to_rgb):
  """
  register RGB image to Depth frame
  :param rgb_img: rgb image with shape height x width x 3 in np array
  :param depth_img: depth image with shape height x width x 3 in np array
  :param rgb_info: rgb camera info msg from ros
  :param depth_info: depth camera info msg from ros
  :param depth_to_rgb: 4x4 transformation matrix from depth frame to rgh frame. note that the z axis should be pointing
    out from the image, x should be pointing right, y should be pointing up
  :return: registered rgb image with shape height x width x 3 in np array
  """
  height, width = depth_img.shape
  reg_rgb_img = np.zeros_like(rgb_img, dtype=np.int)
  inv_depth_fx = 1 / depth_info.K[0]
  inv_depth_fy = 1 / depth_info.K[4]
  depth_cx = depth_info.K[2]
  depth_cy = depth_info.K[5]
  depth_Tx = 0.
  depth_Ty = 0.

  rgb_fx = rgb_info.K[0]
  rgb_fy = rgb_info.K[4]
  rgb_cx = rgb_info.K[2]
  rgb_cy = rgb_info.K[5]
  rgb_Tx = 0.
  rgb_Ty = 0.

  pixel_depth = np.mgrid[0:width, 0:height]
  pixel_depth = np.concatenate((pixel_depth, depth_img.T.reshape(1, width, height)))
  pixel_depth = pixel_depth.reshape(3, -1).T
  xyz_depth = np.copy(pixel_depth)
  xyz_depth[:, 0] = ((xyz_depth[:, 0] - depth_cx) * xyz_depth[:, 2] - depth_Tx) * inv_depth_fx
  xyz_depth[:, 1] = ((xyz_depth[:, 1] - depth_cy) * xyz_depth[:, 2] - depth_Ty) * inv_depth_fy
  xyz_depth = np.concatenate((xyz_depth, np.ones((xyz_depth.shape[0], 1))), 1)

  xyz_rgb = depth_to_rgb.dot(xyz_depth.T).T
  u_rgb = (rgb_fx * xyz_rgb[:, 0] + rgb_Tx) / xyz_rgb[:, 2] + rgb_cx + 0.5
  u_rgb = u_rgb.astype(np.int)
  v_rgb = (rgb_fy * xyz_rgb[:, 1] + rgb_Ty) / xyz_rgb[:, 2] + rgb_cy + 0.5
  v_rgb = v_rgb.astype(np.int)
  valid = np.logical_and.reduce((u_rgb >= 0, u_rgb < width, v_rgb >= 0, v_rgb < height))
  pixel_depth = pixel_depth.astype(np.int)
  reg_rgb_img[pixel_depth[valid, 1], pixel_depth[valid, 0]] = rgb_img[v_rgb[valid], u_rgb[valid]]
  return reg_rgb_img