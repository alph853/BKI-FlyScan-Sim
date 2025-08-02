from setuptools import find_packages, setup
import glob
import os

package_name = 'flyscan_simulation'

# Helper: collect only actual files from worlds/ recursively
install_files = [
    f for f in glob.glob('worlds/**/*', recursive=True)
    if os.path.isfile(f)
] + glob.glob('launch/*')

data_files = [
    ('share/ament_index/resource_index/packages',
     [f'resource/{package_name}']),
    ('share/' + package_name,
     ['package.xml']),
]

# Add each file with correct relative subdirectory path
for file_path in install_files:
    # Get directory relative to 'worlds/'
    relative_dir = os.path.dirname(file_path)
    data_files.append((
        f'share/{package_name}/{relative_dir}',
        [file_path]
    ))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "video_streamer     = flyscan_simulation.video_streamer:main",
            "visualization_node = flyscan_simulation.visualization_node:main",
            "spawn_stock        = flyscan_simulation.spawn_stock:main",
        ],
    },
)
