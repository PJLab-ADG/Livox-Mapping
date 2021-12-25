import pcl
from loguru import logger

import os
import sys
import time
import numpy as np

import cv2
import matplotlib.pyplot as plt
from numba import njit
from numba.cuda import jit
from decimal import Decimal

from mmdet.apis import inference_detector
from matplotlib.patches import Polygon

from shapely.geometry import Point as shapePoint
from shapely.geometry.polygon import Polygon as shapePolygon

np.set_printoptions(precision=10)

CLASSES = ('person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus',
           'train', 'truck', 'boat', 'traffic light', 'fire hydrant',
           'stop sign', 'parking meter', 'bench', 'bird', 'cat', 'dog',
           'horse', 'sheep', 'cow', 'elephant', 'bear', 'zebra', 'giraffe',
           'backpack', 'umbrella', 'handbag', 'tie', 'suitcase', 'frisbee',
           'skis', 'snowboard', 'sports ball', 'kite', 'baseball bat',
           'baseball glove', 'skateboard', 'surfboard', 'tennis racket',
           'bottle', 'wine glass', 'cup', 'fork', 'knife', 'spoon', 'bowl',
           'banana', 'apple', 'sandwich', 'orange', 'broccoli', 'carrot',
           'hot dog', 'pizza', 'donut', 'cake', 'chair', 'couch',
           'potted plant', 'bed', 'dining table', 'toilet', 'tv', 'laptop',
           'mouse', 'remote', 'keyboard', 'cell phone', 'microwave',
           'oven', 'toaster', 'sink', 'refrigerator', 'book', 'clock',
           'vase', 'scissors', 'teddy bear', 'hair drier', 'toothbrush')

MOVABLE_CLASSES = ('person', 'bicycle', 'car', 'motorcycle', 'bus', 'truck', 'bird', 'cat', 'dog',
                   'horse', 'sheep', 'cow', 'elephant', 'bear', 'zebra', 'giraffe',)


def set_log_level(level: str = "TRACE", save_log: bool = False, log_file_name: str = None) -> None:
    logger.remove()
    logger.add(sys.stdout, level=level)

    if save_log:
        if log_file_name is None:
            curr_time = time.ctime(time.time())
        else:
            curr_time = log_file_name
        logger.add(f"/data/test/{curr_time}.log", level=level)


def set_calib_parameters():
    extrinsic = np.array([[0.0265714, -0.99964, 0.00379047, 0.0606143], [-0.00218415, -0.00384985, -0.99999, 0.288651],
                          [0.999644, 0.0265626, -0.00228567, 0.258449], [0, 0, 0, 1]])
    intrinsic = np.array([[1023.37, 0, 922.516], [0, 994.267, 556.527], [0, 0, 1]])

    distortion = np.array([[-0.396302, 0.1187, -0.00558465, 0.00159696]])

    return extrinsic, intrinsic, distortion


def init_model(config=None, check_point=None):
    from mmdet.apis import (init_detector)
    if config is None:
        config = '../configs/libra_rcnn/libra_faster_rcnn_x101_64x4d_fpn_1x_coco.py'
    if check_point is None:
        check_point = '../checkpoints/libra_faster_rcnn_x101_64x4d_fpn_1x_coco_20200315-3a7d0488.pth'

    model = init_detector(config, check_point, device='cuda:0')

    return model


# align the pc_list with image_list
def align_file_lists(img_list, pc_list):
    # image files are full, need to select images according to the odom files

    pc_timestamps = [float(f.split("/")[-1].split(".pcd")[0]) for f in pc_list]
    img_timestamps = [float(s.split("/")[-1].split(".png")[0]) for s in img_list]

    selected_img_list = []

    img_idx = 0
    for i in range(len(pc_list)):
        cur_img_time = img_timestamps[img_idx]
        cur_pc_time = pc_timestamps[i]

        while np.abs(cur_img_time - cur_pc_time) > 0.1:
            if img_idx > len(img_timestamps) - 1:
                break
            img_idx += 1
            cur_img_time = img_timestamps[img_idx]

        selected_img_list.append(img_list[img_idx])

    if len(selected_img_list) != len(pc_list):
        logger.error("The file size is not same!")

    return selected_img_list, pc_list


def files_saving_path(base_path=None):
    if base_path is None:
        base_path = "../demo/inference_result"

    proj_file_path = os.path.join(base_path, "proj")
    if not os.path.exists(proj_file_path):
        os.makedirs(proj_file_path)

    detection_img_path = os.path.join(base_path, "detection")
    if not os.path.exists(detection_img_path):
        os.makedirs(detection_img_path)

    npy_path = os.path.join(base_path, "npy")
    if not os.path.exists(npy_path):
        os.makedirs(npy_path)

    pcd_path = os.path.join(base_path, "pcd")
    if not os.path.exists(pcd_path):
        os.makedirs(pcd_path)

    return proj_file_path, detection_img_path, npy_path, pcd_path


def fetch_dataset(data_base, livox_pc_path=None, img_path=None):
    if livox_pc_path is None:
        livox_pc_path = os.path.join(data_base, "pcd")
    if img_path is None:
        img_path = os.path.join(data_base, "img")

    if not os.path.exists(livox_pc_path):
        raise RuntimeError(f"PCD path {livox_pc_path} does not exist")

    if not os.path.exists(img_path):
        raise RuntimeError(f"Image path {img_path} does not exist")

    image_list, pc_list = [], []
    image_list = [os.path.join(img_path, f) for f in os.listdir(img_path) if ".png" in f]
    image_list.sort(key=lambda s: float(s.split("/")[-1].split(".png")[0]))

    odom_flag = True
    if odom_flag:
        pc_list = [os.path.join(livox_pc_path, f) for f in os.listdir(livox_pc_path) if ".pcd" in f]
        pc_list.sort(key=lambda s: float(
            float(s.split("/")[-1].split(".pcd")[0].split("_")[0])))

        image_list, pc_list = align_file_lists(image_list, pc_list)
    else:
        pc_list = [os.path.join(livox_pc_path, f) for f in os.listdir(livox_pc_path) if ".npy" in f]
        pc_list.sort(key=lambda s: float(
            float(s.split("/")[-1].split(".npy")[0])
        ))

    return pc_list, image_list


def img_undistort(img_ori, intrinsic_param, distort_param):
    intrinsic_param = intrinsic_param[:, :3]
    iden = np.identity(3)
    w, h = img_ori.shape[1], img_ori.shape[0]

    mapx, mapy = cv2.initUndistortRectifyMap(intrinsic_param, distort_param, iden, intrinsic_param, (w, h),
                                             cv2.CV_32FC1)
    img_dst = cv2.remap(img_ori, mapx, mapy, cv2.INTER_LINEAR)

    return img_dst


def convert_rot_trans(extrinsic):
    rot_matrix = extrinsic[0:3, 0:3]
    rot_vec, _ = cv2.Rodrigues(rot_matrix)
    trans_vec = extrinsic[0:3, 3]
    return rot_vec, trans_vec


def project_pts_to_img(pts, image, extrinsic, intrinsic, cam_dist, polygons):
    img_width = image.shape[1]
    img_height = image.shape[0]

    if len(pts) < 10:
        logger.debug("too few points to project to the image")
        return

    if pts.shape[1] < 4:
        pts_ = np.ones((len(pts), 4))
        pts_[:, 0:3] = pts
    else:
        pts_ = pts

    ration = 0.2
    min_w = -ration * img_width
    max_w = (1 + ration) * img_width
    min_h = - ration * img_height
    max_h = (1 + ration) * img_height
    pt_img_ = []
    pts_filtered = []
    for pt in pts_:
        pt_geo = np.copy(pt)
        pt_geo[3] = 1.0
        if np.any(np.isnan(pt)):
            continue

        pt_z = extrinsic[2, 0] * pt_geo[0] + extrinsic[2, 1] * pt_geo[1] + extrinsic[2, 2] * pt_geo[2] + extrinsic[
            2, 3]
        if pt_z < 1.0:
            continue
        pt_cam = np.dot(extrinsic, pt_geo)
        pt_img = np.dot(intrinsic, pt_cam[0:3]
                        / pt_cam[2])
        if pt_img[0] < min_w or pt_img[0] > max_w:
            continue
        if pt_img[1] < min_h or pt_img[1] > max_h:
            continue
        pt_img_.append(pt_geo[0:3])
        pts_filtered.append(pt)
    if len(pt_img_) < 1:
        logger.warning("no point correspondence found between lidar-image")
        return pt_img_, pts_filtered

    rvec, tvec = convert_rot_trans(extrinsic)
    dis_cam = np.asarray(cam_dist)
    proj_pts, _ = cv2.projectPoints(np.asarray(pt_img_), rvec, tvec,
                                    np.asarray(intrinsic), dis_cam)
    proj_pts = proj_pts.reshape(-1, 2)

    inds = np.logical_and(0 < proj_pts[:, 0], proj_pts[:, 0] < img_width)
    proj_pts = proj_pts[inds, :]
    pts_filtered = np.asarray(pts_filtered)[inds, :]

    inds = np.logical_and(0 < proj_pts[:, 1], proj_pts[:, 1] < img_height)
    proj_pts = proj_pts[inds, :]
    pts_filtered = pts_filtered[inds, :]

    flags = np.ones(len(proj_pts), dtype=bool)

    pts_inside = []
    pts_lidar_outside = []
    pts_image_outside = []
    for poly in polygons:
        shape_poly = shapePolygon(poly.get_xy())
        for idx, (pt_lidar, pt_img) in enumerate(zip(pts_filtered, proj_pts)):
            if not flags[idx]:
                continue
            if shape_poly.contains(shapePoint(pt_img[0], pt_img[1])):
                pts_inside.append(pt_img)
                flags[idx] = False

    pts_image_outside = proj_pts[flags]
    pts_lidar_outside = pts_filtered[flags]

    pt_img = np.asarray(pts_image_outside)
    pt_lidar = np.asarray(pts_lidar_outside)
    return pt_img, pt_lidar


def img_process(img, model, intrinsic_param, distort_param, visual_flag=False):
    img = img_undistort(img, intrinsic_param, distort_param)
    inference_result = inference_detector(model, img)

    return img, inference_result


def projection_visualization(img, pts, save_pth=None):
    ax = plt.subplot(111)
    plt.title("Projection")
    plt.imshow(img)
    img_width = img.shape[1]
    img_height = img.shape[0]

    counter = 0
    counter_pt = 0

    if pts is not None and len(pts) > 0:
        for pt in pts:
            if pt[0] < 0 or pt[0] > img_width:
                continue
            if pt[1] < 0 or pt[1] > img_height:
                continue
            counter += 1
            if len(pts) > 5000:
                skip_rate = 20
            else:
                skip_rate = 10
            if counter % skip_rate == 0:
                ax.plot(pt[0], pt[1], "x", c='b')
                counter_pt += 1
            else:
                continue
    if save_pth is None:
        logger.trace(" number of lidar pts found in the image: %d" % counter)
        plt.pause(0.1)
        plt.show()
        plt.clf()
    else:
        plt.savefig(save_pth, dpi=200)
        plt.clf()


def enlarge_bbox(bbox_init, img_size, enlarge_step=10):
    bbox_init[0] = max(bbox_init[0] - enlarge_step, 0)
    bbox_init[1] = max(bbox_init[1] - enlarge_step, 0)
    bbox_init[2] = min(bbox_init[2] + enlarge_step, img_size[0])
    bbox_init[3] = min(bbox_init[3] + enlarge_step, img_size[1])
    return bbox_init


def extract_inference_result(model, img, result, score_thr=0.6):
    if hasattr(model, 'module'):
        model = model.module
    if isinstance(result, tuple):
        bbox_result, segm_result = result
        if isinstance(segm_result, tuple):
            segm_result = segm_result[0]  # ms rcnn
    else:
        bbox_result, segm_result = result, None
    bboxes = np.vstack(bbox_result)
    labels = [
        np.full(bbox.shape[0], i, dtype=np.int32)
        for i, bbox in enumerate(bbox_result)
    ]
    labels = np.concatenate(labels)

    if score_thr > 0:
        assert bboxes.shape[1] == 5
        scores = bboxes[:, -1]
        inds = scores > score_thr
        bboxes = bboxes[inds, :]
        labels = labels[inds]

    polygons = []
    labels_text = []
    w, h = img.shape[1], img.shape[0]
    for i, (bbox, label) in enumerate(zip(bboxes, labels)):
        label_text = CLASSES[label]
        if label_text not in MOVABLE_CLASSES:
            continue
        bbox_int = bbox.astype(np.int32)
        bbox_int = enlarge_bbox(bbox_int, [w, h], enlarge_step=20)
        poly = [[bbox_int[0], bbox_int[1]], [bbox_int[0], bbox_int[3]],
                [bbox_int[2], bbox_int[3]], [bbox_int[2], bbox_int[1]]]
        np_poly = np.array(poly).reshape((4, 2))
        polygons.append(Polygon(np_poly))
        labels_text.append(label_text)

    return polygons, labels_text


def inference_and_remove(pc_list, img_list, inference_model, save_path=None):
    extrinsic, intrinsic, distortion = set_calib_parameters()
    proj_folder, det_folder, npy_folder, pcd_folder = files_saving_path(save_path)

    for pc_, img_ in zip(pc_list, img_list):
        logger.trace("The processed pc:  %s " % pc_)
        logger.trace("The processed img:  %s " % img_)

        img = cv2.imread(img_)
        if ".npy" in pc_:
            pc = np.load(pc_)
        else:
            pc = pcl.load_XYZI(pc_).to_array()

        img_undis, inference_result = img_process(img, inference_model, intrinsic, distortion)
        polygons, labels_text = extract_inference_result(inference_model, img, inference_result)

        pt_img, pt_lidar = project_pts_to_img(pc, img, extrinsic, intrinsic, distortion, polygons)

        img_name = img_[img_.rfind("/") + 1:]
        pc_name = pc_[pc_.rfind("/") + 1: pc_.rfind(".")] + ".npy"

        proj_file_pth = os.path.join(proj_folder, img_name)
        save_img = os.path.join(det_folder, img_name)
        pcd_path = os.path.join(npy_folder, pc_name)
        inference_model.show_result(img, inference_result, score_thr=0.6, out_file=save_img)
        logger.trace("Save trimmed NPY to: %s " % pcd_path)
        np.save(pcd_path, pt_lidar)
        projection_visualization(img, pt_img, proj_file_pth)


def euler_to_rotMat(yaw, pitch, roll):
    Rz_yaw = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw), np.cos(yaw), 0],
        [0, 0, 1]])
    Ry_pitch = np.array([
        [np.cos(pitch), 0, np.sin(pitch)],
        [0, 1, 0],
        [-np.sin(pitch), 0, np.cos(pitch)]])
    Rx_roll = np.array([
        [1, 0, 0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll), np.cos(roll)]])
    # R = RzRyRx
    rotMat = np.dot(Rz_yaw, np.dot(Ry_pitch, Rx_roll))
    return rotMat

def main(inference=False):
    if inference:
        inference_model = init_model()

    data_base = "DEMO"
    pc_path = os.path.join(data_base, "pcd_correction")
    img_path = os.path.join(data_base, "img")
    save_result_path = os.path.join(data_base, "result")

    if inference:
        pc_list, img_list = fetch_dataset(data_base, livox_pc_path=pc_path, img_path=img_path)
        inference_and_remove(pc_list, img_list, inference_model, save_result_path)


if __name__ == "__main__":
    set_log_level()
    main()
