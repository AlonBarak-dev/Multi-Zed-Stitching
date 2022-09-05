import sys
import time

import ogl_viewer.viewer as gl
import pyzed.sl as sl
import threading

if __name__ == "__main__":
    print("Running Depth Sensing sample ... Press 'Esc' to quit")

    init = sl.InitParameters(camera_resolution=sl.RESOLUTION.HD720,
                             depth_mode=sl.DEPTH_MODE.ULTRA,
                             coordinate_units=sl.UNIT.METER,
                             coordinate_system=sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP)

    res = sl.Resolution()
    res.width = 720
    res.height = 404


    def cloud(cam):
        init.set_from_serial_number(cam.serial_number)
        name_list.append("ZED {}".format(cam.serial_number))
        print("Opening {}".format(name_list[index]))
        zed = sl.Camera()
        status = zed.open(init)
        if status != sl.ERROR_CODE.SUCCESS:
            print(repr(status))
            zed.close()

        camera_model = zed.get_camera_information().camera_model
        # Create OpenGL viewer
        viewer = gl.GLViewer()
        viewer.init(len(sys.argv), sys.argv, camera_model, res)
        point_cloud = sl.Mat(res.width, res.height, sl.MAT_TYPE.F32_C4, sl.MEM.CPU)

        while viewer.is_available():
            if zed.grab() == sl.ERROR_CODE.SUCCESS:
                zed.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA, sl.MEM.CPU, res)
                viewer.updateData(point_cloud)

        viewer.exit()
        zed_list[index].close()


    name_list = []
    zed_list = []
    thread_list = []
    cameras = sl.Camera.get_device_list()
    index = 0
    for camera in cameras:
        thread_list.append(threading.Thread(target=cloud, args=(camera,)))
        thread_list[index].start()
        index = index + 1
        time.sleep(20)

