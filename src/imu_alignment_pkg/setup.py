from setuptools import setup

package_name = "imu_alignment_pkg"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/imu_align.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Itsuki Fujiwara",
    maintainer_email="fujiwara-itsuki696@g.ecc.u-tokyo.ac.jp",
    description="Align and convert IMU data from ESP32 into sensor_msgs/Imu",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            # 左がコマンド名、右がPythonモジュール＋関数
            "imu_align_node = imu_alignment_pkg.imu_align_node:main",
        ],
    },
)
