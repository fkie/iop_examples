from setuptools import setup

package_name = 'fkie_iop_cfg_sim_turtle'
setup(
    name=package_name,
    version='1.1.0',
    # package_dir={'': 'src'},
    # packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name + '/launch', ['launch/control_services.yaml',
                                               'launch/control.launch.xml',
                                               'launch/iop_node_manager.launch.xml',
                                               'launch/turtle_services.yaml',
                                               'launch/turtle.launch.xml']),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'fkie_iop_msgs', 'nav_msgs'],
    zip_safe=True,
    author='Alexander Tiderko',
    maintainer='Alexander Tiderko',
    maintainer_email='alexander.tiderko@fkie.fraunhofer.de',
    keywords=['ROS2', 'IOP'],
    description=(
        'Example launch files for robots and OCU.'
    ),
    license='GPLv2',
    tests_require=['pytest'],
    # entry_points={
    #     'console_scripts': [
    #         'rqt_iop_access_control = fkie_iop_rqt_access_control.main:main',
    #     ],
    # },
    scripts = [
        'scripts/turtle2iop.py'
    ]
)