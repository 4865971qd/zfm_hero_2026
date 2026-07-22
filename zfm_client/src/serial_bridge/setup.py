from setuptools import setup

package_name = 'serial_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dev',
    maintainer_email='dev@example.com',
    description='Bridge VideoPacket to 0x0310 serial frames for RM image TX link',
    license='MIT',
    entry_points={
        'console_scripts': [
            'serial_bridge_node = serial_bridge.serial_bridge_node:main',
        ],
    },
)
