from setuptools import find_packages, setup

package_name = 'picar_controller'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/'+package_name]),
        ('share/'+package_name,           ['package.xml']),
        ('share/'+package_name+"/launch", ["launch/controller.launch"]),
        ('share/'+package_name+"/config", ["config/controller.yaml"]),
      ],
    package_data={'': ['py.typed']},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Johanna Pluschke',
    maintainer_email='johanna.pluschke.ext@ptb.de',
    description='ROS 2 main controller for PiCar-X using Sunfounder Robot Hat V3.3',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'state_publisher = picar_controller.state_publisher:main',
            'line_follower = picar_controller.line_follower:main',
            'utility_service = picar_controller.utility_service:main'
        ],
    },
)
