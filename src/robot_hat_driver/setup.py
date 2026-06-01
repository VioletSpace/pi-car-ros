from setuptools import find_packages, setup

package_name = 'robot_hat_driver'

setup(
 name=package_name,
 version='0.0.0',
 packages=find_packages(exclude=['test']),
 data_files=[
     ('share/ament_index/resource_index/packages',
             ['resource/' + package_name]),
     ('share/' + package_name, ['package.xml']),
     ("share/" + package_name + "/launch",
             ["launch/robot_hat.launch.py"]),
     ("share/" + package_name + "/config",
             ["config/robot_hat.yaml"]),
   ],
 install_requires=['setuptools', 'robot_hat'],
 zip_safe=True,
 maintainer='Johanna Pluschke',
 maintainer_email='johanna.pluschke.ext@ptb.de',
 description='ROS 2 interface for SunFounder Robot HAT V3.3',
 license='MIT',
 tests_require=['pytest'],
 entry_points={
     'console_scripts': [
             'robot_hat_node = robot_hat_driver.robot_hat_node:main'
     ],
   },
)