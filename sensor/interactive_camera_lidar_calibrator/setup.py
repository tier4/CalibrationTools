from setuptools import setup

package_name = "interactive_camera_lidar_calibrator"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Kenzo Lobos Tsunekawa",
    maintainer_email="kenzo.lobos@tier4.jp",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "interactive_calibrator = interactive_camera_lidar_calibrator.interactive_calibrator:main"
        ],
    },
)
