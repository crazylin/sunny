from setuptools import setup, find_packages

package_name = 'seam_tracking'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, 'seam_tracking.ros2_numpy'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='zhuoqiw@hotmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'main = seam_tracking.seam_tracking_node:main',
        ],
    },
)
