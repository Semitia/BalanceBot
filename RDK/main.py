import yaml


def load_config(filename):
    with open(filename, 'r') as file:
        return yaml.safe_load(file)


# 加载配置文件
config = load_config('config.yaml')