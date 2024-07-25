from dataclasses import dataclass, fields
from time import time_ns
from kap_dataclass.data_utils import generate_byte_array


@dataclass
class BrakeCtrl:
    brake_en_ctrl: int = 0
    aeb_en_ctrl: int = 0
    brake_dec: int = 0
    brake_pedal_target: int = 0
    checksum: int = 0

    def update_value(self, **kwargs) -> None:

        for field in fields(self):
            if field.name in kwargs:
                value = kwargs[field.name]
                setattr(self, field.name, value)

    def get_value(self, field_name):

        if not hasattr(self, field_name):
            raise AttributeError(f"'BrakeCtrlData' object has no attribute '{field_name}'")
        return getattr(self, field_name)

    def add_cycle_count(self):

        if self.checksum >= 255:
            self.checksum = 0

        self.checksum += 1

    def get_bytearray(self):
        brake_en_ctrl = (self.brake_en_ctrl, 0, 0)
        aeb_en_ctrl = (self.aeb_en_ctrl, 1, 1)
        brake_dec_upper = ((self.brake_dec >> 8) & 0xFF, 8, 15)
        brake_dec_lower = (self.brake_dec & 0b00000011, 22, 23)
        brake_pedal_target_upper = ((self.brake_pedal_target >> 8) & 0xFF, 24, 31)
        brake_pedal_target_lower = (self.brake_pedal_target & 0xFF, 32, 39)
        checksum = (self.checksum, 56, 63)

        return generate_byte_array(8,
                                   brake_en_ctrl,
                                   aeb_en_ctrl,
                                   brake_dec_upper,
                                   brake_dec_lower,
                                   brake_pedal_target_upper,
                                   brake_pedal_target_lower,
                                   checksum,
                                   xor_checksum=False)
