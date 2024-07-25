import math

from rclpy.node import Node

from tier4_vehicle_msgs.msg import BatteryStatus
from autoware_auto_vehicle_msgs.msg import (ControlModeReport, GearReport, HazardLightsReport, TurnIndicatorsReport,
                                            SteeringReport, VelocityReport)
from sensor_msgs.msg import Imu
import can
import rclpy
import threading
from struct import unpack

from kap_dataclass.DriveStaFb import DriveStaFb
from kap_dataclass.BrakeStaFb import BrakeStaFb
from kap_dataclass.SteerStaFb import SteerStaFb
from kap_dataclass.VehicleStaFb import VehicleStaFb
from kap_dataclass.VehicleWorkStaFb import VehicleWorkStaFb
from kap_dataclass.PowerStaFb import PowerStaFb

# not use but to do
from kap_dataclass.VehicleFltSta import VehicleFltSta
from kap_dataclass.WheelRpmFb import WheelRpmFb


class CANReceiverNode(Node):
    """
    Node responsible for receiving CAN messages and publishing them as ROS2 messages.

    TODO: Check publisher queue size.
    """

    def __init__(self):
        super().__init__('CANReportNode')

        self.bus = can.interface.Bus(channel='can0', bustype='socketcan')
        self.running = False
        self.receive_thread = threading.Thread(target=self.receive_data)

        # Report message object
        # self.msg_obj_battery_rpt = BatteryStatus()
        # self.msg_obj_control_mode_rpt = ControlModeReport()
        # self.msg_obj_gear_rpt = GearReport()
        # self.msg_obj_hazardLights_rpt = HazardLightsReport()
        # self.msg_obj_indicators_rpt = TurnIndicatorsReport()
        # self.msg_obj_steering_rpt = SteeringReport()
        # self.msg_obj_velocity_rpt = VelocityReport()
        # self.msg_obj_imu = Imu()

        # Report data class
        self.drive_sta_fb = DriveStaFb()
        self.brake_sta_fb = BrakeStaFb()
        self.steer_sta_fb = SteerStaFb()
        self.vehicle_sta_fb = VehicleStaFb()
        self.vehicle_work_sta_fb = VehicleWorkStaFb()
        self.power_sta_fb = PowerStaFb()

        # vehicle status report publisher
        # self.battery_rpt_publisher = self.create_publisher(BatteryStatus, '/vehicle/status/battery_charge', 10)
        # self.control_mode_rpt_publisher = self.create_publisher(ControlModeReport, '/vehicle/status/control_mode', 10)
        # self.gear_rpt_publisher = self.create_publisher(GearReport, '/vehicle/status/gear_status', 10)
        # self.hazard_lights_rpt_publisher = self.create_publisher(HazardLightsReport,
        #                                                          '/vehicle/status/hazard_lights_status', 10)
        # self.turn_indicators_rpt_publisher = self.create_publisher(TurnIndicatorsReport,
        #                                                            '/vehicle/status/turn_indicators_status', 10)
        # self.steering_rpt_publisher = self.create_publisher(SteeringReport, '/vehicle/status/steering_status', 10)
        # self.velocity_rpt_publisher = self.create_publisher(VelocityReport, '/vehicle/status/velocity_status', 100)

        # subscriber
        # self.imu_subscriber = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)

        # publisher timer

        # self.throttle_ctrl_send_timer = self.create_timer(0.02, self.throttle_ctrl_data_timer_callback)
        # self.battery_rpt_timer = self.create_timer(0.02, self.battery_rpt_timer_callback)
        # self.control_mode_rpt_timer = self.create_timer(0.02, self.control_mode_rpt_timer_callback)
        # self.gear_rpt_timer = self.create_timer(0.02, self.gear_rpt_timer_callback)
        # self.hazard_lights_rpt_timer = self.create_timer(0.02, self.hazard_lights_rpt_timer_callback)
        # self.turn_indicators_rpt_timer = self.create_timer(0.02, self.turn_indicators_rpt_timer_callback)
        # self.steering_rpt_timer = self.create_timer(0.02, self.steering_rpt_timer_callback)
        # self.velocity_rpt_timer = self.create_timer(0.02, self.velocity_rpt_timer_callback)

        # global member init
        self.heading_rate = 0.00
        self.start()

    def start(self):
        self.running = True
        self.receive_thread.start()

    def stop(self):
        self.running = False
        self.receive_thread.join()

    def imu_callback(self, msg):
        self.heading_rate = msg.angular_velocity.z

    def receive_data(self):
        """
        Continuously receives CAN message and processes them.
        """
        while self.running:
            try:
                message = self.bus.recv()
                can_id = message.arbitration_id
                data = message.data

                if message.is_error_frame:
                    pass
                # TODO : If the message contains an error frame, additional code for data processing is required.
                if message.is_remote_frame:
                    pass
                # TODO : Requires processing when the message contains a remote request frame
                if not message.is_extended_id and message.dlc == 8 and not message.is_remote_frame:
                    self.process_can_data_and_publish(can_id, data)
            except can.CanError as _:
                pass

    def process_can_data_and_publish(self, can_id, data):
        # Byte Order : little endian = intel endian
        # Drive Status Feedback Parsing
        if can_id == 0x530:
            driver_en_sta = unpack('<B', data[0:1])[0] & 0b00000001
            diver_slop_over = unpack('<B', data[0:1])[0] & 0b00000010
            driver_mode_Sta = unpack('<B', data[0:1])[0] & 0b00000110
            gear_fb = unpack('<B', data[0:1])[0] & 0b00110000
            speed_fb = unpack('<h', data[1:3])[0] * 0.01
            throttle_pald_fb = (unpack('<H', data[3:5])[0] & 0b0000001111111111) * 0.1
            acceleration_fb = unpack('<b', data[5:7])[0] * 0.01
            drive_life = unpack('<B', data[7:8])[0] & 0b00001111

            data = {
                'driver_en_sta': driver_en_sta,
                'diver_slop_over': diver_slop_over,
                'driver_mode_Sta': driver_mode_Sta,
                'gear_fb': gear_fb,
                'speed_fb': speed_fb,
                'throttle_pald_fb': throttle_pald_fb,
                'acceleration_fb': acceleration_fb,
                'drive_life': drive_life,
            }
            self.get_logger().info('Received CAN message with ID {}'.format(can_id))
            self.get_logger().info('Received CAN message with data {}'.format(data))
            self.throttle_rpt_data.update_value(**data)

        # # Brake Status FeedBack Parsing
        # elif can_id == 0x531:
        #
        #     data = {
        #         'brake_en_state': brake_en_state,
        #         'brake_flt1': brake_flt1,
        #         'brake_flt2': brake_flt2,
        #         'brake_pedal_actual': brake_pedal_actual
        #     }
        #
        #     self.brake_rpt_data.update_value(**data)
        #
        # # Steering Status FeedBack Parsing
        # elif can_id == 0x532:
        #
        #     data = {
        #         'steer_en_state': steer_en_state,
        #         'steer_flt1': steer_flt1,
        #         'steer_flt2': steer_flt2,
        #         'steer_angle_actual': steer_angle_actual,
        #         'steer_angle_spd_actual': steer_angle_spd_actual
        #     }
        #
        #     self.steer_rpt_data.update_value(**data)
        #
        # # Vehicle Work Status FeedBack Parsing
        # elif can_id == 0x534:
        #
        #     data = {
        #         'gear_actual': gear_actual_rpt,
        #         'gear_flt': gear_flt
        #     }
        #
        #     self.gear_rpt_data.update_value(**data)
        #
        #     # Power Status FeedBack Parsing
        #
        #     data = {
        #         'parking_actual': parking_actual,
        #         'park_flt': park_flt
        #     }
        #
        #     self.park_rpt_data.update_value(**data)
        #
        # # Vehicle Status FeedBack Parsing
        # # TODO : Check AEB_STATE Report
        # elif can_id == 0x536:
        #
        #     data = {
        #         'acc': acc,
        #         'brake_light_actual': brake_light_actual,
        #         'steer_mode_sts': steer_mode_sts,
        #         'speed': speed,
        #         'drive_mode_sts': drive_mode_sts,
        #         'vehicle_mode_sts': vehicle_mode_sts_rpt,
        #         'back_crash_state': back_crash_state,
        #         'front_crash_state': front_crash_state,
        #         'aeb_state': aeb_state,
        #         'chassis_errcode': chassis_errcode,
        #         'turn_light_actual': turn_light_actual_rpt,
        #         'hazard_light_actual_rpt': hazard_light_actual_rpt,
        #
        #     }
        #
        #     self.vcu_rpt_data.update_value(**data)
        #
        # # Vehicle Fault Status Parsing
        # elif can_id == 0x537:
        #
        #     data = {
        #         'front_left': front_left,
        #         'front_right': front_right,
        #         'rear_left': rear_left,
        #         'rear_right': rear_right
        #     }
        #
        #     self.wheel_spd_rpt_data.update_value(**data)
        #
        # # Chassis Wheel Rpm FeedBack Parsing
        # elif can_id == 0x539:
        #
        #     data = {
        #         'batter_voltage': batter_voltage,
        #         'battery_current': battery_current,
        #         'battery_soc': battery_soc
        #     }
        #
        #     self.bms_rpt_data.update_value(**data)

    # def throttle_ctrl_data_timer_callback(self):
    #     pass

    # def battery_rpt_timer_callback(self):
    #     self.msg_obj_battery_rpt.energy_level = float(self.bms_rpt_data.get_value('battery_soc'))
    #     self.battery_rpt_publisher.publish(self.msg_obj_battery_rpt)
    #
    # def control_mode_rpt_timer_callback(self):
    #     self.msg_obj_control_mode_rpt.mode = self.vcu_rpt_data.get_value('vehicle_mode_sts')
    #     self.control_mode_rpt_publisher.publish(self.msg_obj_control_mode_rpt)
    #
    # def hazard_lights_rpt_timer_callback(self):
    #     self.msg_obj_hazardLights_rpt.report = self.vcu_rpt_data.get_value('hazard_light_actual')
    #     self.hazard_lights_rpt_publisher.publish(self.msg_obj_hazardLights_rpt)
    #
    # def turn_indicators_rpt_timer_callback(self):
    #     self.msg_obj_indicators_rpt.report = self.vcu_rpt_data.get_value('turn_light_actual')
    #     self.turn_indicators_rpt_publisher.publish(self.msg_obj_indicators_rpt)
    #
    # def steering_rpt_timer_callback(self):
    #     self.msg_obj_steering_rpt.steering_tire_angle = self.steer_rpt_data.get_value('steer_angle_actual')
    #     self.steering_rpt_publisher.publish(self.msg_obj_steering_rpt)
    #
    # def gear_rpt_timer_callback(self):
    #     self.msg_obj_gear_rpt.report = self.gear_rpt_data.get_value('gear_actual')
    #     self.gear_rpt_publisher.publish(self.msg_obj_gear_rpt)
    #
    # def velocity_rpt_timer_callback(self):
    #     self.msg_obj_velocity_rpt.header.stamp = self.get_clock().now().to_msg()
    #     self.msg_obj_velocity_rpt.header.frame_id = "base_link"
    #     self.msg_obj_velocity_rpt.longitudinal_velocity = float(self.vcu_rpt_data.get_value('speed'))
    #     self.msg_obj_velocity_rpt.heading_rate = self.heading_rate
    #     self.velocity_rpt_publisher.publish(self.msg_obj_velocity_rpt)


def main(args=None):
    rclpy.init(args=args)
    node = CANReceiverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
