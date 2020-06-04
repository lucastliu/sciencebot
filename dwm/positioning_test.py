import threading

from DWMTag import DWMTag
from DWMBot import DWMBot


def main():
    bot = DWMBot()
    while True:
        position = bot.get_position()
        print('position: ', position)


main()