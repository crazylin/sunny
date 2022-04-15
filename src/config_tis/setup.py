from setuptools import setup

package_name = 'config_tis'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/.params.yaml', 'config/params.yaml']),
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
            'config_tis_node = config_tis.config_tis_node:main',
        ],
    },
)
