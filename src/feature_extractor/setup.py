from setuptools import find_packages, setup

package_name = 'feature_extractor'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sudil-minthaka',
    maintainer_email='sudilminthaka8797@gmail.com',
    description='Extracts image feature vectors from camera frames using MobileNetV3.',
    license='TODO: License declaration',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'feature_extractor = feature_extractor.feature_extractor_node:main',
        ],
    },
)
