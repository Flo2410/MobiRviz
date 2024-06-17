#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import json

from mobi_interfaces.srv import GetCalibStatus, GetImuCalibData


class SaveIMUCalib(Node):

    def __init__(self):
        super().__init__('save_imu_calib')
        self.get_logger().info("SaveIMUCalib Node run")

        self.imu_get_calib_status = self.create_client(GetCalibStatus, "imu_get_calib_status")
        while not self.imu_get_calib_status.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('imu_get_calib_status service not available, waiting again...')
        self.get_imu_calib_status_req = GetCalibStatus.Request()

        self.imu_get_calib_data = self.create_client(GetImuCalibData, "imu_get_calib_data")
        while not self.imu_get_calib_data.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('imu_get_calib_data service not available, waiting again...')
        self.imu_get_calib_data_req = GetImuCalibData.Request()

    def check_imu_calib_status(self):
        return self.imu_get_calib_status.call_async(self.get_imu_calib_status_req)

    def get_imu_calib_data(self):
        return self.imu_get_calib_data.call_async(self.imu_get_calib_data_req)

    
def main(args=None):
    rclpy.init(args=args)
    client = SaveIMUCalib()

    # Status
    future_calib_status = client.check_imu_calib_status()
    rclpy.spin_until_future_complete(client, future_calib_status)
    calib_status = future_calib_status.result()
    client.get_logger().info(f"{calib_status}")

    # Data
    future_calib_data = client.get_imu_calib_data()
    rclpy.spin_until_future_complete(client, future_calib_data)
    calib_data = future_calib_data.result()
    # client.get_logger().info(f"{calib_data}")
    
    data = {
        'offset_gyroscope_x': calib_data.offset_gyroscope_x,
        'offset_gyroscope_y': calib_data.offset_gyroscope_y,
        'offset_gyroscope_z': calib_data.offset_gyroscope_z,
        'offset_magnetometer_x': calib_data.offset_magnetometer_x,
        'offset_magnetometer_y': calib_data.offset_magnetometer_y,
        'offset_magnetometer_z': calib_data.offset_magnetometer_z,
        'offset_accelerometer_x': calib_data.offset_accelerometer_x,
        'offset_accelerometer_y': calib_data.offset_accelerometer_y,
        'offset_accelerometer_z': calib_data.offset_accelerometer_z,
        'radius_magnetometer': calib_data.radius_magnetometer,
        'radius_accelerometer': calib_data.radius_accelerometer,
    }

    with open('imu_calib_data.json', 'w', encoding='utf-8') as f:
        json.dump(data, f, ensure_ascii=False, indent=2)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()