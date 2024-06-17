#!/usr/bin/env python3

from time import sleep
import rclpy
from rclpy.node import Node
import json

from mobi_interfaces.srv import GetCalibStatus, SetImuCalibData


class LoadIMUCalib(Node):

    def __init__(self):
        super().__init__('save_imu_calib')
        self.get_logger().info("LoadIMUCalib Node run")

        self.imu_get_calib_status = self.create_client(GetCalibStatus, "imu_get_calib_status")
        while not self.imu_get_calib_status.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('imu_get_calib_status service not available, waiting again...')
        self.get_imu_calib_status_req = GetCalibStatus.Request()

        self.imu_set_calib_data = self.create_client(SetImuCalibData, "imu_set_calib_data")
        while not self.imu_set_calib_data.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('imu_set_calib_data service not available, waiting again...')
        self.imu_set_calib_data_req = SetImuCalibData.Request()

    def check_imu_calib_status(self):
        return self.imu_get_calib_status.call_async(self.get_imu_calib_status_req)

    def set_imu_calib_data(self, data_json):
        print(data_json)
        self.imu_set_calib_data_req.offset_gyroscope_x = data_json.get("offset_gyroscope_x")
        self.imu_set_calib_data_req.offset_gyroscope_y = data_json.get("offset_gyroscope_y")
        self.imu_set_calib_data_req.offset_gyroscope_z = data_json.get("offset_gyroscope_z")
        self.imu_set_calib_data_req.offset_magnetometer_x = data_json.get("offset_magnetometer_x")
        self.imu_set_calib_data_req.offset_magnetometer_y = data_json.get("offset_magnetometer_y")
        self.imu_set_calib_data_req.offset_magnetometer_z = data_json.get("offset_magnetometer_z")
        self.imu_set_calib_data_req.offset_accelerometer_x = data_json.get("offset_accelerometer_x")
        self.imu_set_calib_data_req.offset_accelerometer_y = data_json.get("offset_accelerometer_y")
        self.imu_set_calib_data_req.offset_accelerometer_z = data_json.get("offset_accelerometer_z")
        self.imu_set_calib_data_req.radius_magnetometer = data_json.get("radius_magnetometer")
        self.imu_set_calib_data_req.radius_accelerometer = data_json.get("radius_accelerometer")

        return self.imu_set_calib_data.call_async(self.imu_set_calib_data_req)
    
def main(args=None):
    rclpy.init(args=args)
    client = LoadIMUCalib()

    with open('imu_calib_data.json', "r", encoding="utf-8") as data_file:
      data_loaded = json.load(data_file)

    # Data
    future_calib_data = client.set_imu_calib_data(data_json=data_loaded)
    rclpy.spin_until_future_complete(client, future_calib_data)
    res = future_calib_data.result()
    client.get_logger().info(f"{res}")

    sleep(1)

    # Status
    future_calib_status = client.check_imu_calib_status()
    rclpy.spin_until_future_complete(client, future_calib_status)
    calib_status = future_calib_status.result()
    client.get_logger().info(f"{calib_status}")

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()