import os

from setuptools import setup

package_name = "new_extrinsic_calibration_manager"

data_files = [
    ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
    ("share/" + package_name, ["package.xml"]),
]


def package_files(data_files, directory_list):
    paths_dict = {}

    for directory in directory_list:
        for path, _, filenames in os.walk(directory):
            for filename in filenames:
                file_path = os.path.join(path, filename)
                install_path = os.path.join("share", package_name, path)

                if install_path in paths_dict.keys():
                    paths_dict[install_path].append(file_path)

                else:
                    paths_dict[install_path] = [file_path]

    for key in paths_dict.keys():
        data_files.append((key, paths_dict[key]))

    return data_files


setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=package_files(data_files, ["launch/"]),
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Kenzo Lobos Tsunekawa",
    maintainer_email="kenzo.lobos@tier4.jp",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "new_extrinsic_calibration_manager = new_extrinsic_calibration_manager.new_extrinsic_calibration_manager:main"
        ],
    },
)
