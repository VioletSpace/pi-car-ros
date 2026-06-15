from setuptools import find_packages, setup

package_name = 'picar_steamdeck'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/'+package_name]),
        ('share/'+package_name,                       ['package.xml']),
        ('share/'+package_name+"/launch",             ["launch/system.launch"]),
        ('share/'+package_name+"/config",             ["config/steamdeck.yaml"]),
    ],
    package_data={'': ['py.typed']},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Johanna Freya Pluschke',
    maintainer_email='johanna.pluschke.ext@ptb.de',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
