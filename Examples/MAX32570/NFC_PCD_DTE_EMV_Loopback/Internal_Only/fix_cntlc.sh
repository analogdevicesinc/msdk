#!/bin/sh
kill -STOP `ps | grep gdb | awk '{print $1}'`
