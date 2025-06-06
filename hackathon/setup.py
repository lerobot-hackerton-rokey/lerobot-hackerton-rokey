from setuptools import find_packages, setup

package_name = 'hackathon'

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
            'joint_state_sub = hackathon.joint_state_subscriber:main',
            'ik_client = hackathon.ik_client:main',
            'move_joint = hackathon.move_joint:main',
            'move_with_joint_command_6dof = hackathon.move_with_joint_command_6dof:main',
            'move_with_joint_command_7dof = hackathon.move_with_joint_command_7dof:main',
        ],
    },
)
