import logging

class LogHelper:
    logger = logging.getLogger()
    hdlr = logging.FileHandler('main.log')
    formatter = logging.Formatter('%(asctime)s %(levelname)s %(message)s')
    hdlr.setFormatter(formatter)
    logger.addHandler(hdlr) 
    logger.setLevel(logging.DEBUG)

    def logError(self,message):
        print(message)
        logging.exception(message)
    
    def logInfo(self,message):
        print(message)
        logging.info(message)

    
