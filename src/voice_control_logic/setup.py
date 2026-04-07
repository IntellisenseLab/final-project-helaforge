from setuptools import find_packages, setup
import os

package_name = 'voice_control_logic'

# Include models directory in package data
data_files = [
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
]

# Add models directory to data files
models_dir = os.path.join(os.path.dirname(__file__), 'voice_control_logic', 'models')
if os.path.isdir(models_dir):
    for root, dirs, files in os.walk(models_dir):
        install_path = os.path.join('share', package_name, os.path.relpath(root, os.path.join(os.path.dirname(__file__), 'voice_control_logic')))
        files_list = [os.path.join(root, f) for f in files]
        if files_list:
            data_files.append((install_path, files_list))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools', 'ament_index_python'],
    zip_safe=True,
    maintainer='sudil-minthaka',
    maintainer_email='sudilminthaka8797@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'voice_interpreter = voice_control_logic.voice_interpreter:main',
        ],
    },
)
