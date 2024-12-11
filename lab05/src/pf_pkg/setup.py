from setuptools import find_packages, setup

package_name = 'pf_pkg'

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
    maintainer='thilevin',
    maintainer_email='thilevin@todo.todo',
    description='Paricle Filter package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pf_node = pf_pkg.pfpkg:main',
            'odom_fix = pf_pkg.odomFix_node:main',
        ],
    },
)
