from setuptools import find_packages, setup

package_name = 'dsr_example'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gossi',
    maintainer_email='mincheol710313@gmail.com',
    description='TODO: Package description',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'dance = dsr_example.demo.dance_m1013:main',
                'single_robot_simple = dsr_example.simple.single_robot_simple:main',
                'slope_demo = dsr_example.demo.slope_demo:main',
                'example_comp = dsr_example.simple.example_comp:main',
                'example_force = dsr_example.simple.example_force:main',
                'example_gripper = dsr_example.simple.example_gripper:main',
                'gripper_drl_controller = dsr_example.simple.gripper_drl_controller:main',
                'example_move = dsr_example.simple.example_move:main',
                'test_realsense = dsr_example.simple.test_realsense:main',
                'test_realsense_force = dsr_example.simple.test_realsense_force:main',
            ],
        },
    )
