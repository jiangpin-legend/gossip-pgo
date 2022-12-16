import logging
import datetime

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

class LoggerAdapter(logging.LoggerAdapter):
    def __init__(self, logger, prefix):
        super(LoggerAdapter, self).__init__(logger, {})
        self.prefix = prefix

    def process(self, msg, kwargs):
        return '[%s] %s' % (self.prefix, msg), kwargs
  
class CustomLogger(logging.Logger):
  def __init__(self, name: str, level) -> None:
    super().__init__(name, level)
    self.fmt = '[%(levelname)-8s:  %(filename)s:%(lineno)d in function %(funcName)s] %(message)s'
    self.stdout_handler = logging.StreamHandler()
    self.stdout_handler.setLevel(level)
    self.stdout_handler.setFormatter(CustomFormatter(self.fmt))
    self.addHandler(self.stdout_handler)

    # today = datetime.date.today()
    # self.file_handler = logging.FileHandler('my_app_{}.log'.format(today.strftime('%Y_%m_%d')))
    # self.file_handler.setLevel(logging.DEBUG)
    # self.file_handler.setFormatter(logging.Formatter(CustomFormatter(self.fmt)))
    # self.addHandler(self.file_handler)



class MultiRobotLogger():
  def __init__(self, name, robot_id,level):
    custom_logger = CustomLogger(__name__,logging.INFO)
    robot_id = 0
    self.logger = LoggerAdapter(custom_logger, 'robot_'+str(robot_id))

  



if __name__=='__main__':
  custom_logger = CustomLogger(__name__,logging.INFO)
  robot_id = 0
  logger = LoggerAdapter(custom_logger, 'robot_'+str(robot_id))

  logger.info('hello')
  logger.warning('hello')

  # mr_logger = MultiRobotLogger(__name__,2,logging.DEBUG)
  # mr_logger.debug('hello')