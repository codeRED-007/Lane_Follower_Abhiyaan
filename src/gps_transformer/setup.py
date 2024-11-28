from setuptools import find_packages, setup

package_name = 'gps_transformer'

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
    maintainer='yoogi',
    maintainer_email='yoogikov@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'transformer=gps_transformer.transformer:main',
            'sim=gps_transformer.simulator:main',
            'gps_publisher=gps_transformer.goal_publisher:main'
        ],
    },
)
