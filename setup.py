from setuptools import find_packages, setup

package_name = 'turtle_controller'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'opencv-python', 'numpy'],
    zip_safe=True,
    description='Package for controlling turtle with camera feed and ArUco markers',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "test_node = turtle_controller.first_node:main"
        ],
    },
)