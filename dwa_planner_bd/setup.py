from setuptools import find_packages, setup

package_name = 'dwa_planner_bd'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Bhavya Dalal',
    maintainer_email='dalalbhavya@gmail.com',
    description='TODO: [Refine] Custom DWA Planner for mobile robots',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dwa_planner = dwa_planner_bd.dwa_planner:main'
        ],
    },
)
