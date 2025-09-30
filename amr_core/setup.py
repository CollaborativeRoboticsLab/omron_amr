from setuptools import setup

package_name = 'amr_core'

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
    maintainer='omron',
    maintainer_email='guanyewtan@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'action_server = amr_core.action_server:main',
            'goto_goal = amr_core.goto_goal:main',
            'goto_goal_demo = amr_core.goto_goal_demo:main',
            'dock = amr_core.dock:main',
            'goto_point = amr_core.goto_point:main',
            'localize_at_point = amr_core.localize_at_point:main',
            'execute_macro = amr_core.execute_macro:main',
            'arcl_api_server = amr_core.arcl_api_server:main',
            'ld_states_publisher = amr_core.ld_states_publisher:main',
        ],
    },
)
