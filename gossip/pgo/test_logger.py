import logging

# logging.basicConfig(format='%(asctime)s,%(msecs)03d %(levelname)-8s [%(filename)s:%(lineno)d] %(message)s',
#     datefmt='%Y-%m-%d:%H:%M:%S',
#     level=logging.DEBUG)

class CustomFormatter(logging.Formatter):
    """Logging colored formatter, adapted from https://stackoverflow.com/a/56944256/3638629"""

    grey = '\x1b[38;21m'
    blue = '\x1b[38;5;39m'
    yellow = '\x1b[38;5;226m'
    red = '\x1b[38;5;196m'
    bold_red = '\x1b[31;1m'
    reset = '\x1b[0m'

    def __init__(self, fmt):
        super().__init__()
        self.fmt = fmt
        self.FORMATS = {
            logging.DEBUG: self.grey + self.fmt + self.reset,
            logging.INFO: self.blue + self.fmt + self.reset,
            logging.WARNING: self.yellow + self.fmt + self.reset,
            logging.ERROR: self.red + self.fmt + self.reset,
            logging.CRITICAL: self.bold_red + self.fmt + self.reset
        }

    def format(self, record):
        log_fmt = self.FORMATS.get(record.levelno)
        formatter = logging.Formatter(log_fmt)
        return formatter.format(record)


fmt = '[%(levelname)-8s:  %(filename)s:%(lineno)d in function %(funcName)s] %(message)s'
stdout_handler = logging.StreamHandler()
stdout_handler.setLevel(logging.DEBUG)
stdout_handler.setFormatter(CustomFormatter(fmt))
logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)
logger.addHandler(stdout_handler)
# logging.basicConfig(format='[%(levelname)-8s:  %(filename)s:%(lineno)d in function %(funcName)s] %(message)s',
#     level=logging.DEBUG)

def this_func(a):
  logger.debug(a)
  print(a)

logger = logging.getLogger(__name__)
logger.debug("This is a debug log")
this_func(1)
logger.info("This is an info log")
logger.warning("This is a warning log")
logger.critical("This is critical")
logger.error("An error occurred")