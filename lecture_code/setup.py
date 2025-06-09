from setuptools import find_packages, setup

package_name = 'lecture_code'

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
    maintainer='we',
    maintainer_email='llaayy.kr@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "move_gear = lecture_code.move_gear:main",
            "move_block = lecture_code.move_block:main",
            "stack_upgrade = lecture_code.stack_upgrade:main",
            "task1 = lecture_code.task1:main",
            
        ],
    },
)
