import pyzed.sl as sl
import cv2
import numpy as np
import threading
import time
import signal
import matplotlib as plt

zed_list = []
left_list = []
depth_list = []
timestamp_list = []
thread_list = []
stop_signal = False
left_ind = 0
right_ind = 0
left_img = None
right_img = None

def signal_handler(signal, frame):
    global stop_signal
    stop_signal = True
    time.sleep(0.5)
    exit()


def ShowDisparity(bsize=25):
    global left_img
    global right_img
    global zed_list
    while True:
        try:
            depth_img_right = sl.Mat()
            depth_img_left = sl.Mat()
            zed_list[0].retrieve_measure(depth_img_right, sl.MEASURE.DEPTH)
            zed_list[1].retrieve_measure(depth_img_left, sl.MEASURE.DEPTH)
            depth_img_rgb_right = depth_img_right.get_data()
            cv2.imwrite('rgba_right.jpg', depth_img_rgb_right)
            depth_img_rgb_left = depth_img_right.get_data()
            cv2.imwrite('rgba_left.jpg', depth_img_rgb_left)
            depth_img_left = cv2.imread('rgba_left.jpg')
            depth_img_left = cv2.cvtColor(depth_img_left, cv2.COLOR_GRAY2RGB)
            depth_img_right = cv2.imread('rgba_right.jpg')
            depth_img_right = cv2.cvtColor(depth_img_right, cv2.COLOR_GRAY2RGB)
            stitch_list = [depth_img_left, depth_img_right]
            stitcher = cv2.Stitcher_create()
            (status, stitched) = stitcher.stitch(stitch_list)
            # if the status is '0', then OpenCV successfully performed image
            # stitching
            print(status)
            if status == 0:
                # display the output stitched image to our screen
                stitched = cv2.resize(stitched, (0, 0), fx=0.4, fy=0.4)
                cv2.imshow("Stitched", stitched)
                cv2.waitKey(1)

        except Exception:
            pass


def stitch_images():
    global stop_signal
    global zed_list
    global timestamp_list
    global left_list
    global depth_list
    global left_ind
    global right_ind
    global right_img
    global left_img

    # if left_img is not None and right_img is not None:
    while True:
        try:
            stitch_list = [left_img, right_img]
            print("[INFO] stitching images...")
            stitcher = cv2.Stitcher_create()

            (status, stitched) = stitcher.stitch(stitch_list)

            # if the status is '0', then OpenCV successfully performed image
            # stitching
            if status == 0:
                # display the output stitched image to our screen
                stitched = cv2.resize(stitched, (0, 0), fx=0.4, fy=0.4)
                cv2.imshow("Stitched", stitched)
                cv2.waitKey(1)
            # otherwise the stitching failed, likely due to not enough keypoints)
            # being detected
            else:
                print("[INFO] image stitching failed ({})".format(status))
        except Exception:
            pass


def grab_run(index):
    global stop_signal
    global zed_list
    global timestamp_list
    global left_list
    global depth_list
    global left_ind
    global right_ind
    global right_img
    global left_img

    runtime = sl.RuntimeParameters()

    while not stop_signal:
        err = zed_list[index].grab(runtime)
        if err == sl.ERROR_CODE.SUCCESS:
            if index == 0:  # left view
                zed_list[index].retrieve_image(left_list[index], sl.VIEW.LEFT)
                # cv2.imshow("left", left_list[index].get_data())
                # key = cv2.waitKey(10)
                # path = "DATA_TWO/LEFT/left_" + str(left_ind) + ".jpg"
                path = "left_.jpg"
                print("saved image at " + path)
                cv2.imwrite(path, left_list[index].get_data())
                left_ind += 1
                left_img = cv2.imread("left_.jpg")
            else:
                zed_list[index].retrieve_image(left_list[index], sl.VIEW.RIGHT)
                # cv2.imshow("right", left_list[index].get_data())
                # key = cv2.waitKey(10)
                # path = "DATA_TWO/RIGHT/right_" + str(right_ind) + ".jpg"
                path = "right_.jpg"
                print("saved image at " + path)
                cv2.imwrite(path, left_list[index].get_data())
                right_ind += 1
                right_img = cv2.imread("right_.jpg")

            zed_list[index].retrieve_measure(depth_list[index], sl.MEASURE.DEPTH)
            timestamp_list[index] = zed_list[index].get_timestamp(sl.TIME_REFERENCE.CURRENT).data_ns

        time.sleep(0.001)  # 1ms
    zed_list[index].close()


def main():
    global stop_signal
    global zed_list
    global left_list
    global depth_list
    global timestamp_list
    global thread_list
    signal.signal(signal.SIGINT, signal_handler)

    print("Running...")
    init = sl.InitParameters()
    init.camera_resolution = sl.RESOLUTION.HD1080
    init.camera_fps = 30  # The framerate is lowered to avoid any USB3 bandwidth issues

    # List and open cameras
    name_list = []
    last_ts_list = []
    cameras = sl.Camera.get_device_list()
    index = 0
    for cam in cameras:
        init.set_from_serial_number(cam.serial_number)
        name_list.append("ZED {}".format(cam.serial_number))
        print("Opening {}".format(name_list[index]))
        zed_list.append(sl.Camera())
        left_list.append(sl.Mat())
        depth_list.append(sl.Mat())
        timestamp_list.append(0)
        last_ts_list.append(0)
        status = zed_list[index].open(init)
        if status != sl.ERROR_CODE.SUCCESS:
            print(repr(status))
            zed_list[index].close()
        index = index + 1

    # Start camera threads
    for index in range(0, len(zed_list)):
        if zed_list[index].is_opened():
            thread_list.append(threading.Thread(target=grab_run, args=(index,)))
            thread_list[index].start()

    # stitch_thread = threading.Thread(target=stitch_images)
    # stitch_thread.start()

    disparity_thread = threading.Thread(target=ShowDisparity)
    disparity_thread.start()

    # Display camera images
    key = ''
    while key != 113:  # for 'q' key
        for index in range(0, len(zed_list)):
            if zed_list[index].is_opened():
                if timestamp_list[index] > last_ts_list[index]:
                    # cv2.imshow(name_list[index], left_list[index].get_data())
                    x = round(depth_list[index].get_width() / 2)
                    y = round(depth_list[index].get_height() / 2)
                    err, depth_value = depth_list[index].get_value(x, y)
                    if np.isfinite(depth_value):
                        print("{} depth at center: {}MM".format(name_list[index], round(depth_value)))
                    last_ts_list[index] = timestamp_list[index]
        key = cv2.waitKey(10)
    cv2.destroyAllWindows()

    # Stop the threads
    stop_signal = True
    for index in range(0, len(thread_list)):
        thread_list[index].join()

    print("\nFINISH")


if __name__ == "__main__":
    main()
