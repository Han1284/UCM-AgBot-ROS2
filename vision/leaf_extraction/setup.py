from setuptools import setup

package_name = 'leaf_extraction'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (
            'share/' + package_name + '/segmentation_model',
            ['segmentation_model/citrus.pt'],
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mehrad Mortazavi',
    maintainer_email='smortazavi3@ucmerced.edu',
    description='Image processing pipeline for leaf canopy extraction',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'instance_segmentation = leaf_extraction.instance_segmentation:main',
            'segmentation_preview = leaf_extraction.segmentation_preview:main',
            'collect_sim_dataset = leaf_extraction.collect_sim_dataset:main',
        ],
    },
)
