from rclpy.node import Node
import rclpy
from time import time_ns
import math

from can_utils.can_sender import CANSender

from autoware_auto_vehicle_msgs.msg import GearCommand
from tier4_control_msgs.msg import GateMode
from tier4_vehicle_msgs.msg import VehicleEmergencyStamped
from autoware_auto_vehicle_msgs.msg import TurnIndicatorsCommand
from autoware_auto_vehicle_msgs.msg import HazardLightsCommand
from tier4_vehicle_msgs.msg import ActuationCommandStamped

from kap_dataclass.ThrottleCtrlData import ThrottleCtrlData
from kap_dataclass.BrakeCtrlData import BrakeCtrlData
from kap_dataclass.SteerCtrlData import SteerCtrlData
from kap_dataclass.GearCtrlData import GearCtrlDatav
from kap_dataclass.ParkCtrlData import ParkCtrlData
from kap_dataclass.VehicleModeCtrlData import VehicleModeCtrlData


class CANCommandNode(Node):

    def __init__(self):
        super().__init__('CANCommandNode')
        self.can_channel = 'can0'
        self.can_sender = CANSender(self.can_channel)

        self.sub_gear_ctrl_cmd = self.create_subscription(GearCommand, '/control/command/gear_cmd',
                                                          self.dispatch_command, 10)

        self.sub_gate_mode_ctrl_cmd = self.create_subscription(GateMode, '/control/current_gate_mode',
                                                               self.dispatch_command, 10)

        self.sub_vehicle_emergency_ctrl_cmd = self.create_subscription(VehicleEmergencyStamped,
                                                                       '/control/command/emergency_cmd',
                                                                       self.dispatch_command, 10)

        self.sub_turn_indicators_ctrl_cmd = self.create_subscription(TurnIndicatorsCommand,
                                                                     '/control/command/turn_indicators_cmd',
                                                                     self.dispatch_command, 10)

        self.sub_turn_hazard_lights_ctrl_cmd = self.create_subscription(HazardLightsCommand,
                                                                        '/control/command/hazard_lights_cmd',
                                                                        self.dispatch_command, 10)

        self.sub_actuation = self.create_subscription(ActuationCommandStamped,
                                                      '/control/command/actuation_cmd',
                                                      self.dispatch_command, 10)

        self.throttle_ctrl_send_timer = self.create_timer(0.02, self.throttle_ctrl_data_timer_callback)
        self.brake_ctrl_send_timer = self.create_timer(0.02, self.brake_ctrl_data_timer_callback)
        self.steer_ctrl_send_timer = self.create_timer(0.02, self.steer_data_timer_callback)
        self.gear_ctrl_send_timer = self.create_timer(0.05, self.gear_ctrl_data_timer_callback)
        self.park_ctrl_send_timer = self.create_timer(0.02, self.park_ctrl_data_timer_callback)
        self.vehicle_mode_ctrl_send_timer = self.create_timer(0.1, self.vehicle_mode_ctrl_data_timer_callback)

        self.throttle_ctrl_data = ThrottleCtrlData()
        self.brake_ctrl_data = BrakeCtrlData()
        self.steer_ctrl_data = SteerCtrlData()
        self.gear_ctrl_data = GearCtrlData()
        self.park_ctrl_data = ParkCtrlData()
        self.vehicle_mode_ctrl_data = VehicleModeCtrlData()

        self.set_throttle_ctrl_default_value()
        self.set_brake_ctrl_default_value()
        self.set_steering_ctrl_default_value()
        self.set_gear_ctrl_default_value()
        self.set_park_ctrl_default_value()
        self.set_vehicle_mode_ctrl_default_value()

    def set_throttle_ctrl_default_value(self):

        throttle_ctrl_cmd_data = {
            'throttle_en_ctrl': 1,
            'throttle_acc': 0,
            'throttle_pedal_target': 0,
            'vel_target': 0,
        }
        self.throttle_ctrl_data.update_value(**throttle_ctrl_cmd_data)
        self.can_sender.send(0x100, self.throttle_ctrl_data.get_bytearray())

    def set_brake_ctrl_default_value(self):
        brake_ctrl_cmd_data = {
            'brake_en_ctrl': 1,
            'aeb_en_ctrl': 1,
            'brake_dec': 0,
            'brake_pedal_target': 0,
        }
        self.brake_ctrl_data.update_value(**brake_ctrl_cmd_data)
        self.can_sender.send(0x101, self.brake_ctrl_data.get_bytearray())

    def set_steering_ctrl_default_value(self):

        steer_ctrl_cmd_data = {
            'steer_en_ctrl': 1,
            'steer_angle_spd': 480,
            'steer_angle_target': 500,
        }
        self.steer_ctrl_data.update_value(**steer_ctrl_cmd_data)
        self.can_sender.send(0x102, self.steer_ctrl_data.get_bytearray())

    def set_gear_ctrl_default_value(self):

        gear_ctrl_cmd_data = {
            'gear_en_ctrl': 1,
            'gear_target': 3,
        }

        self.gear_ctrl_data.update_value(**gear_ctrl_cmd_data)
        self.can_sender.send(0x103, self.gear_ctrl_data.get_bytearray())

    def set_park_ctrl_default_value(self):

        park_ctrl_cmd_data = {
            'park_en_ctrl': 1,
            'park_target': 0,
        }

        self.park_ctrl_data.update_value(**park_ctrl_cmd_data)
        self.can_sender.send(0x104, self.park_ctrl_data.get_bytearray())

    def set_vehicle_mode_ctrl_default_value(self):

        vehicle_mode_ctrl_cmd_data = {
            'steer_mode_ctrl': 0,
            'drive_mode_ctrl': 0,
            'turn_light_ctrl': 0,
        }

        self.vehicle_mode_ctrl_data.update_value(**vehicle_mode_ctrl_cmd_data)
        self.can_sender.send(0x105, self.vehicle_mode_ctrl_data.get_bytearray())


    def dispatch_command(self, msg):

        if isinstance(msg, ActuationCommandStamped):
            command_data_throttle = {
                'throttle_pedal_target': int(msg.actuation.accel_cmd / 0.01),
            }
            command_data_brake = {
                'brake_pedal_target': int(msg.actuation.brake_cmd / 0.01),
            }
            command_data_steering = {
                'steer_angle_target': int(((math.degrees(msg.actuation.steer_cmd)) * (500 / 30)) + 500),
            }

            self.throttle_ctrl_data.update_value(**command_data_throttle)
            self.brake_ctrl_data.update_value(**command_data_brake)
            self.steer_ctrl_data.update_value(**command_data_steering)

        elif isinstance(msg, GearCommand):
            # INVALID
            gear_command = 0
            # NEUTRAL
            if msg.command == 1:
                gear_command = 3
            # DRIVE
            elif 2 <= msg.command <= 19:
                gear_command = 4
            # REVERSE
            elif 20 <= msg.command <= 21:
                gear_command = 2
            # PARK : TODO
            elif msg.command == 22:
                pass
            # Default N
            else:
                gear_command = 3

            gear_ctrl_cmd_data = {
                'gear_target': gear_command,
            }

            self.gear_ctrl_data.update_value(**gear_ctrl_cmd_data)
        elif isinstance(msg, GateMode):
            pass

        elif isinstance(msg, VehicleEmergencyStamped):
            # TODO: Apply emergency braking system and parking brake using BrakeCtrlDataclass and DriveCtrlDataClass.
            pass

        elif isinstance(msg, TurnIndicatorsCommand):
            turn_light = 0

            if msg.command == 2:
                turn_light = 1
            elif msg.command == 3:
                turn_light = 2
            else:
                turn_light = 0

            command_data = {
                'turn_light_ctrl': turn_light,
            }

            self.vehicle_mode_ctrl_data.update_value(**command_data)

        elif isinstance(msg, HazardLightsCommand):

            turn_light = 0

            if msg.command == 2:
                turn_light = 3

            command_data = {
                'turn_light_ctrl': turn_light,
            }

            self.vehicle_mode_ctrl_data.update_value(**command_data)

    def throttle_ctrl_data_timer_callback(self):
        self.throttle_ctrl_data.add_cycle_count()
        self.can_sender.send(0x100, self.throttle_ctrl_data.get_bytearray())

    def brake_ctrl_data_timer_callback(self):
        self.brake_ctrl_data.add_cycle_count()
        self.can_sender.send(0x101, self.brake_ctrl_data.get_bytearray())

    def steer_data_timer_callback(self):
        self.steer_ctrl_data.add_cycle_count()
        self.can_sender.send(0x102, self.steer_ctrl_data.get_bytearray())

    def gear_ctrl_data_timer_callback(self):
        self.gear_ctrl_data.add_cycle_count()
        self.can_sender.send(0x103, self.gear_ctrl_data.get_bytearray())

    def park_ctrl_data_timer_callback(self):
        self.park_ctrl_data.add_cycle_count()
        self.can_sender.send(0x104, self.park_ctrl_data.get_bytearray())

    def vehicle_mode_ctrl_data_timer_callback(self):
        self.vehicle_mode_ctrl_data.add_cycle_count()
        self.can_sender.send(0x105, self.vehicle_mode_ctrl_data.get_bytearray())



def main(args=None):
    rclpy.init(args=args)
    node = CANCommandNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
