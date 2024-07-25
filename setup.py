from setuptools import find_packages, setup
import os
from glob import glob

# Other imports ...


package_name = 'kap_vehicle_interface'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('pix_vehicle_interface/launch', '*launch.[py]*'))),
        ('lib/' + package_name + '/can_utils', ['kap_vehicle_interface/can_utils/can_sender.py']),
        ('lib/' + package_name + '/pix_dataclass', ['kap_vehicle_interface/kap_dataclass/data_utils.py',
                                                    'kap_vehicle_interface/kap_dataclass/BrakeCtrlData.py',
                                                    'kap_vehicle_interface/kap_dataclass/data_utils.py',
                                                    'kap_vehicle_interface/kap_dataclass/GearCtrlData.py',
                                                    'kap_vehicle_interface/kap_dataclass/ParkCtrlData.py',
                                                    'kap_vehicle_interface/kap_dataclass/SteerCtrlData.py',
                                                    'kap_vehicle_interface/kap_dataclass/ThrottleCtrlData.py',
                                                    'kap_vehicle_interface/kap_dataclass/VehicleModeCtrlData.py',
                                                    'kap_vehicle_interface/kap_dataclass/BMSReportData.py',
                                                    'kap_vehicle_interface/kap_dataclass/BrakeReportData.py',
                                                    'kap_vehicle_interface/kap_dataclass/GearReportData.py',
                                                    'kap_vehicle_interface/kap_dataclass/ParkReportData.py',
                                                    'kap_vehicle_interface/kap_dataclass/SteerReportData.py',
                                                    'kap_vehicle_interface/kap_dataclass/ThrottleReportData.py',
                                                    'kap_vehicle_interface/kap_dataclass/VcuReportData.py',
                                                    'kap_vehicle_interface/kap_dataclass/WheelSpeedReportData.py']),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='JEON YANG HO',
    maintainer_email='yhjeon@avgenius.kr',
    description='kap vehicle interface',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'kap_interface_rpt = kap_vehicle_interface.kap_interface_rpt_node:main',
            'kap_interface_cmd = kap_vehicle_interface.kap_interface_cmd_node:main',
            'kap_interface_test = kap_vehicle_interface.test_cmd:main',
        ],
    },
)
