import sys
import os

__author__ = "Benjamin VINOT <benjamin.vinot@maximintegrated.com>"

VERBOSE = 1
EXTRA_VERBOSE = 2
DEBUG = 3

class Error(Exception):
    """Base class for exceptions in this module."""
    pass


class FlashCRKWriteError(Error):
    """Base class for exceptions in this module."""
    pass


class FileError(Error):
    """Base class for exceptions in this module."""
    pass


class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'


def print_err(text):
    print >> sys.stderr, bcolors.FAIL + text + bcolors.ENDC


def print_ok(text):
    print bcolors.OKGREEN + text + bcolors.ENDC


def get_fullpath(file_dir, file_name):
    if file_dir == "":
        return file_name
    if os.name == "posix":
        return file_dir + '/' + file_name
    if os.name == "nt":
        return file_dir + '\\' + file_name


def mpc_chip_info(usn, phase):
    print '{:0>2X}{:0>2X}{:0>26X}{:0>2X}'.format(2, 9, int(usn, 16), int(phase))


def mpc_status(current, total):
    print '{:0>2X}{:0>2X}{:0>4X}{:0>4X}'.format(1, 4, current, total)


def mpc_error(module, err):
    print '{:0>2X}{:0>2X}{:0>4X}{:0>4X}'.format(1, 4, module, err)
