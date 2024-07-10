import yaml
import logging

class ConfigLoader:
    @staticmethod
    def load(config_path):
        with open(config_path, 'r') as file:
            return yaml.safe_load(file)

class Logger:
    @staticmethod
    def info(message):
        logging.info(message)
