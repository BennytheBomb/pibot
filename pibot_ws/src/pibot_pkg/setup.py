from setuptools import find_packages, setup

package_name = 'pibot_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'flask'],
    zip_safe=True,
    maintainer='benito',
    maintainer_email='benito@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
			'ball_follow_driver = pibot_pkg.ball_follow_driver:main',
            'ball_follow_tracker = pibot_pkg.ball_follow_tracker:main',
            'driver_demo = pibot_pkg.driver_demo:main',
            'video_streamer = pibot_pkg.video_streamer:main',
        ],
    },
)
