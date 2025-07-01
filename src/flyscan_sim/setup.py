from setuptools import setup

package_name = 'flyscan_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='TTD',
    maintainer_email='ttd@flyscan.com',
    description='Flyscan Simulation Package',
    entry_points={
        'console_scripts': [
            'px4_demo_flight = flyscan_sim.video_streamer:main',
            'drone_controller = flyscan_sim.drone_controller:main',
        ],
    },
)
