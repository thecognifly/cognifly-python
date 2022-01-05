from cognifly.cognifly_remote.cognifly_remote import Cognifly

import logging

logfilename = "cognifly.log"
logfilemode = 'a'
loglevel = 'INFO'

logging.basicConfig(format="[%(levelname)s] [%(asctime)s]: %(message)s",
                    filename=logfilename,
                    filemode=logfilemode,
                    level=getattr(logging, loglevel.upper()))

