from setuptools import setup

package_name = "intrinsic_camera_calibrator"

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
    description="The intrinsic camera calibrator",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "camera_calibrator = intrinsic_camera_calibrator.camera_calibrator:main"
        ],
    },
)
