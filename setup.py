from setuptools import setup, find_packages

setup(
    name='horus_sdk',
    version='0.0.1',
    description='Python SDK for the HORUS Mixed Reality Application',
    long_description=open("README.md").read(),
    long_description_content_type='text/markdown',
    author='Omotoye Shamsudeen Adekoya',
    author_email='omotoye.adekoya@edu.unige.it',
    url='https://github.com/Omotoye/horus_sdk.git',
    
    packages=find_packages(),  # Finds your 'horus' package
    install_requires=[
        'rospy',    # Add other dependencies here
        'pyyaml',
        # Any external dependencies for your SDK
    ],
    python_requires='>=3.7',
)
