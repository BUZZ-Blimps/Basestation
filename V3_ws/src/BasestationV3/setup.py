from setuptools import setup

package_name = 'BasestationV3'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='coreblimp1',
    maintainer_email='swampblimps@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'HelloWorld = BasestationV3.HelloWorld:main',
        	'Main = BasestationV3.Main:main',
        ],
    },
)
