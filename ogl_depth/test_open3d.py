import time

import open3d as o3d
import pyzed.sl as sl
import cv2
import numpy as np
import threading

def main():
    # init = sl.InitParameters(camera_resolution=sl.RESOLUTION.HD720,
    #                          depth_mode=sl.DEPTH_MODE.ULTRA,
    #                          coordinate_units=sl.UNIT.METER,
    #                          coordinate_system=sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP)
    init = sl.InitParameters()
    init.camera_resolution = sl.RESOLUTION.HD720
    init.camera_fps = 30
    zed = sl.Camera()
    # init.set_from_serial_number(34769939)
    status = zed.open(init)
    if status != sl.ERROR_CODE.SUCCESS:
        print(repr(status))
        exit()

    res = sl.Resolution()
    res.width = 720
    res.height = 404
    time.sleep(5)
    rgb_img = sl.Mat()
    depth_img = sl.Mat(zed.get_camera_information().camera_resolution.width,
                             zed.get_camera_information().camera_resolution.height, sl.MAT_TYPE.U8_C4)

    zed.retrieve_image(rgb_img, sl.VIEW.LEFT)
    zed.retrieve_measure(depth_img, sl.MEASURE.DEPTH)

    cv2.imwrite("rgba.jpg", rgb_img.get_data())
    cv2.imwrite("depth.jpg", depth_img.get_data())

    rgb_img_cv = cv2.imread("rgba.jpg")
    depth_img_cv = cv2.imread("depth.jpg")
    print(rgb_img_cv)


























if __name__ == "__main__":
    main()


