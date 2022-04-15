from setuptools import setup

package_name = 'seam_tracking'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/params.yaml']),
        ('lib/python3.8/site-packages/' + package_name, [package_name + '/codes.json']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zhuoqiw',
    maintainer_email='zhuoqiw@hotmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'seam_tracking_node = seam_tracking.seam_tracking_node:main',
        ],
    },
)
