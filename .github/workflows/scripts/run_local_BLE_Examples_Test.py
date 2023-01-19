#!/usr/bin/env python3
# -*- coding:utf-8 -*-

# @Filename: ${NAME}.py

from datetime import datetime as dt
import os
from subprocess import call, Popen, PIPE, CalledProcessError, STDOUT
import sys
import time


ERR_CODE = {
    0: "0",  # NO ERROR
    1: "INVALID ARGUMENTS",
    11: "INVALID FILE",
    12: "EXCEPTION IN SUBPROCESS"
}


def run_file(file_name: str, args:dict) -> int:
    """run a script file

        @:param

    """
    if not os.path.exists(file_name):
        return 11

    do_max32655 = 0
    if "max32655" in args.keys():
        if args["max32655"]:
            do_max32655 = 1
    
    do_max32665 = 0
    if "max32665" in args.keys():
        if args["max32665"]:
            do_max32665 = 1

    do_max32690_evkit = 0
    if "max32690_evkit" in args.keys():
        if args["max32690_evkit"]:
            do_max32690_evkit = 1

    do_max32690_wlp = 0
    if "max32690_wlp" in args.keys():
        if args["max32690_wlp"]:
            do_max32690_wlp = 1

    try:
        file_name = os.path.realpath(file_name)
        p = Popen([f'{file_name} {do_max32655} {do_max32665} {do_max32690_evkit} {do_max32690_wlp} 2>&1 | tee temp.log'], 
                    stdout=PIPE, stderr=PIPE, shell=True)

        for line in iter(p.stdout.readline, b''):
            print(f'{dt.now()} - {line.strip().decode("utf-8")}')
        
        p.stdout.close()
        p.wait()

        return p.returncode

    except Exception as e:
        print(f'Error: {e}')
        p.stdout.close()
        return 12


def main(file_name: str, args: dict):
    print("----------------------------------------------------------------------------------------")
    ret = run_file(file_name, args)

    if ret in ERR_CODE.keys():
        print(f'Return: {ERR_CODE[ret]}')
    else:
        print(f'Return unknown error: {ret}.')

    print("----------------------------------------------------------------------------------------")
    print("Done!")

if __name__ == "__main__":
    file = "local_BLE_Examples_Test.sh"

    args = {"max32655": False, "max32665": False, "max32690_evkit": True, "max32690_wlp": False}

    main(file, args)
