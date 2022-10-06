#!/bin/bash
export MAXIM_PATH="/c/MaximSDK_2_5_2020"
doxygen.exe doxyfile
echo "\n\nContents of doxy_warnings.log:"
cat doxy_warnings.log
