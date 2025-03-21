from setuptools import setup, find_packages

setup(
    name="prain",
    version="1.0.0",
    packages=find_packages(where="src"),
    package_dir={"": "src"},
    install_requires=[
        "opencv-python>=4.5.0",
        "prain_uart>=1.0.0",
        "pyserial>=3.5",
    ],
    python_requires=">=3.10",
    description="High-level controller for our self-driving car on a Raspberry Pi 4",
    author="PREN Team 10",
    author_email="currently.empty@gmail.com",
)
