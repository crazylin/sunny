from setuptools import setup, find_packages

package_name = 'seam_tracking'
depends_name = 'seam_tracking.ros2_numpy'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, depends_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('lib/python3.8/site-packages/' + package_name, [package_name + '/codes.json']),
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
