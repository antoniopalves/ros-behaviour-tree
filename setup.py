from setuptools import setup

package_name = 'trsa_bt_plugins'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/trsa_bt_plugins']),
        ('share/trsa_bt_plugins', ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='antonio',
    maintainer_email='antonio@example.com',
    description='TRSA plugins',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'init_pose_publisher = trsa_bt_plugins.init_pose_publisher:main'
        ],
    },
)
