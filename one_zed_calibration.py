import pyzed.sl as sl
import cv2
import numpy as np
import threading
import time
import signal
import pyboof as pb


zed = sl.Camera()
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


def stitch_images():
    global stop_signal
    global zed
    global timestamp_list
    global left_list
    global depth_list
    global left_ind
    global right_ind
    global right_img
    global left_img

    # if left_img is not None and right_img is not None:
    while True:
        # left_img_ = cv2.imread("left_.jpg")
        # right_img_ = cv2.imread("right_.jpg")
        try:
            # left_img = cv2.resize(left_img, (0, 0), fx=0.4, fy=0.4)
            # right_img = cv2.resize(right_img, (0, 0), fx=0.4, fy=0.4)
            stitch_list = [left_img, right_img]
            print("[INFO] stitching images...")
            stitcher = cv2.Stitcher_create()

            (status, stitched) = stitcher.stitch(stitch_list)

            # if the status is '0', then OpenCV successfully performed image
            # stitching
            if status == 0:
                # write the output stitched image to disk
                # cv2.imwrite("test.jpg", stitched)
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


def grab_run():
    global stop_signal
    global zed
    global timestamp_list
    global left_list
    global depth_list
    global left_ind
    global right_ind
    global right_img
    global left_img

    runtime = sl.RuntimeParameters()
    while not stop_signal:
        err = zed.grab(runtime)
        if err == sl.ERROR_CODE.SUCCESS:
            zed.retrieve_image(left_list[0], sl.VIEW.LEFT)
            # cv2.imshow("left", left_list[0].get_data())
            # key = cv2.waitKey(10)
            # path = "DATA/LEFT/left_" + str(left_ind) + ".jpg"
            path = "left_.jpg"
            print("saved image at " + path)
            cv2.imwrite(path, left_list[0].get_data())
            left_ind += 1

            zed.retrieve_image(left_list[0], sl.VIEW.RIGHT)
            # cv2.imshow("right", left_list[0].get_data())
            # key = cv2.waitKey(10)
            # path = "DATA/RIGHT/right_" + str(right_ind) + ".jpg"
            path = "right_.jpg"
            print("saved image at " + path)
            cv2.imwrite(path, left_list[0].get_data())
            right_ind += 1

            right_img = cv2.imread("right_.jpg")
            left_img = cv2.imread("left_.jpg")

            zed.retrieve_measure(depth_list[0], sl.MEASURE.DEPTH)
            timestamp_list[0] = zed.get_timestamp(sl.TIME_REFERENCE.CURRENT).data_ns

        time.sleep(0.001)  # 1ms
    zed.close()


def main():
    global stop_signal
    global zed
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
    init.set_from_serial_number(cameras[0].serial_number)
    name_list.append("ZED {}".format(cameras[0].serial_number))
    print("Opening {}".format(name_list[index]))
    left_list.append(sl.Mat())
    depth_list.append(sl.Mat())
    timestamp_list.append(0)
    last_ts_list.append(0)
    status = zed.open(init)
    if status != sl.ERROR_CODE.SUCCESS:
        print(repr(status))
        zed.close()

    # Start camera threads
    if zed.is_opened():
        thread_list.append(threading.Thread(target=grab_run))
        thread_list[0].start()

    stitch_thread = threading.Thread(target=stitch_images)
    stitch_thread.start()

    # Display camera images
    key = ''
    while key != 113:  # for 'q' key
        if zed.is_opened():
            if timestamp_list[0] > last_ts_list[0]:
                # cv2.imshow(name_list[0], left_list[0].get_data())
                x = round(depth_list[0].get_width() / 2)
                y = round(depth_list[0].get_height() / 2)
                err, depth_value = depth_list[0].get_value(x, y)
                if np.isfinite(depth_value):
                    print("{} depth at center: {}MM".format(name_list[0], round(depth_value)))
                last_ts_list[0] = timestamp_list[0]
        key = cv2.waitKey(10)
    cv2.destroyAllWindows()

    # Stop the threads
    stop_signal = True
    for index in range(0, len(thread_list)):
        thread_list[index].join()

    print("\nFINISH")


if __name__ == "__main__":
    main()
