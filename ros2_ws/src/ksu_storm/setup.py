from setuptools import find_packages, setup


package_name = "ksu_storm"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}", ["README.md"]),
        (f"share/{package_name}/launch", ["launch/robot.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="cenzell",
    maintainer_email="cenzell@example.com",
    description="Initial ROS 2 Jazzy port for the KSU Storm robot runtime.",
    license="Proprietary",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "robot_node = ksu_storm.robot_node:main",
        ],
    },
)
