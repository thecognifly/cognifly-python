import sys
import os
import io
from setuptools import setup, find_packages

if sys.version_info < (3, 7):
    sys.exit('Sorry, Python < 3.7 is not supported.')


def is_raspberrypi():
    try:
        with io.open('/sys/firmware/devicetree/base/model', 'r') as m:
            if 'raspberry pi' in m.read().lower():
                return True
    except Exception:
        pass
    return False


def is_coral():
    try:
        with io.open('/sys/firmware/devicetree/base/model', 'r') as m:
            if 'coral' in m.read().lower():
                return True
    except Exception:
        pass
    return False


def is_khadas():
    try:
        with io.open('/sys/firmware/devicetree/base/model', 'r') as m:
            if 'khadas' in m.read().lower():
                return True
    except Exception:
        pass
    return False


with open("README.md", "r") as fh:
    long_description = fh.read()

deps = ['numpy', 'Pillow', 'psutil']
if is_raspberrypi():  # drone
    deps.append(['picamera', 'yamspy', 'opencv-python'])
elif is_coral() or is_khadas():  # drone
    deps.append(['yamspy', 'opencv-python'])
else:  # remote
    deps.append(['opencv-contrib-python', 'pysimplegui'])

setup(name='cognifly',
      packages=[package for package in find_packages()],
      version='0.3.1',
      license='MIT',
      description='Control the CogniFly open-source drone from python',
      long_description=long_description,
      long_description_content_type="text/markdown",
      author='Yann Bouteiller',
      url='https://github.com/thecognifly/cognifly-python',
      download_url='https://github.com/thecognifly/cognifly-python/archive/refs/tags/v0.3.1.tar.gz',
      keywords=['cognifly', 'drone', 'remote', 'control'],
      install_requires=deps,
      scripts=["scripts/cognifly-controller", ],
      classifiers=[
          'Development Status :: 4 - Beta',
          'Intended Audience :: Developers',
          'Intended Audience :: Education',
          'Intended Audience :: Information Technology',
          'Intended Audience :: Science/Research',
          'License :: OSI Approved :: MIT License',
          'Operating System :: Microsoft :: Windows',
          'Operating System :: POSIX :: Linux',
          'Programming Language :: Python',
          'Framework :: Robot Framework :: Library',
          'Topic :: Education',
          'Topic :: Scientific/Engineering :: Artificial Intelligence',
      ],
      package_data={'cognifly': [
          'cognifly_remote/sprites/gamepad_off.png',
          'cognifly_remote/sprites/gamepad_on.png']}
      )
