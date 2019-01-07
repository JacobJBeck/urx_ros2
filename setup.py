from setuptools import setup

package_name = 'urx_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[],
    py_modules=['scripts.tracker'],
    data_files=[
        ('share/' + package_name + '/launch', ['launch/ur5.launch.py'])
    ],
    install_requires=['setuptools'],
    author='Jacob Beck',
    author_email='beck@madeinspace.us',
    maintainer='Jacob Beck',
    maintainer_email='beck@madeinspace.us',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: TBD',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='TODO',
    license='TBD',
    entry_points={
        'console_scripts': [
            'tracker = scripts.tracker:main'
        ],
    },
)
