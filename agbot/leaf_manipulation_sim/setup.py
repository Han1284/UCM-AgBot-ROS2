from setuptools import setup

package_name = 'leaf_manipulation_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mehrad Mortazavi',
    maintainer_email='smortazavi3@ucmerced.edu',
    description='Isolated fixed-arm Gazebo simulation for leaf manipulation.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'leaf_pose_adapter = leaf_manipulation_sim.leaf_pose_adapter:main',
            'mock_leaf_pose_publisher = leaf_manipulation_sim.mock_leaf_pose_publisher:main',
            'leaf_grasp_demo = leaf_manipulation_sim.leaf_grasp_demo:main',
        ],
    },
)
