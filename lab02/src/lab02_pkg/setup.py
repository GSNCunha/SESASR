from setuptools import find_packages, setup

package_name = 'lab02_pkg'

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
    maintainer='lincunha',
    maintainer_email='gabriel.sadigursky@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
            'console_scripts': [
                    'talker = lab02_pkg.publisher_member_function:main',
                    'listener = lab02_pkg.subscriber_member_function:main',
            ],
    },
)
