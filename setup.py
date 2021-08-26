from setuptools import setup, find_packages


def is_raspberrypi():
    try:
        with io.open('/sys/firmware/devicetree/base/model', 'r') as m:
            if 'raspberry pi' in m.read().lower(): return True
    except Exception: pass
    return False


with open("README.md", "r") as fh:
    long_description = fh.read()

deps = ['numpy', ]
if is_raspberrypi():
    deps.append(['picamera'])
else:
    deps.append(['opencv-contrib-python'])

setup(name='cognifly',
      packages=[package for package in find_packages()],
      version='0.0.1',
      license='MIT',
      description='Control the CogniFly open-source drone from python',
      long_description=long_description,
      long_description_content_type="text/markdown",
      author='Yann Bouteiller',
      url='',
      download_url='',
      keywords=['cognifly', 'drone', 'remote', 'control'],
      install_requires=deps,
      classifiers=[
          'Development Status :: 5 - Production/Stable',
          'Intended Audience :: Developers',
          'Intended Audience :: Education',
          'Intended Audience :: Information Technology',
          'Intended Audience :: Science/Research',
          'License :: OSI Approved :: MIT License',
          'Operating System :: Microsoft :: Windows',
          'Operating System :: POSIX',
          'Programming Language :: Python',
          'Topic :: Scientific/Engineering :: Artificial Intelligence',  # change this
      ],
      )
