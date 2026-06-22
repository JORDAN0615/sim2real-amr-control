from glob import glob
from setuptools import setup


package_name = "apriltag_amr"


setup(
    name=package_name,
    version="0.1.0",
    py_modules=[
        "demo_loop_runner",
        "mission_logic",
        "multi_camera_object_mission",
    ],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="jordan",
    maintainer_email="jordan@example.com",
    description="ROS 2 AMR demo nodes for multi-camera object mission orchestration.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "demo_loop_runner = demo_loop_runner:main",
            "multi_camera_object_mission = multi_camera_object_mission:main",
        ],
    },
)
