from setuptools import setup

package_name = 'oakd_pointcloud'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/oakd_pointcloud.launch.py']),
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Dada',
    maintainer_email='dada@ics.forth.gr',
    description='OAK-D RGB, Depth and PointCloud2 publisher',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'oakd_pointcloud_publisher = oakd_pointcloud.oakd_pointcloud_publisher:main',
        ],
    },
)
