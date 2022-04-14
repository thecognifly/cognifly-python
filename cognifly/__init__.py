"""
Python API and bluetooth controller for the CogniFly drone.

Install with pip: pip install cognifly
"""

__author__ = "Yann Bouteiller, Ricardo de Azambuja"
__copyright__ = "Copyright 2022, MISTLab.ca"
__credits__ = [""]
__license__ = "MIT"
__version__ = "0.2.0"
__maintainer__ = "Yann Bouteiller"
__email__ = ""
__status__ = "Development"


from cognifly.cognifly_remote.cognifly_remote import Cognifly
import logging


logfilename = "cognifly.log"
logfilemode = 'a'
loglevel = 'INFO'

logging.basicConfig(format="[%(levelname)s] [%(asctime)s]: %(message)s",
                    filename=logfilename,
                    filemode=logfilemode,
                    level=getattr(logging, loglevel.upper()))
