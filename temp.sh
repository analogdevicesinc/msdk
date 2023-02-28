#!/bin/bash
shopt=$(shopt -p nullglob)    # Save nullglob setting
shopt -s nullglob             # Enable nullglob

# Directories to watch
declare -a watched=(
    .github/workflows/ci-tests/Examples_tests
    Examples/MAX32655/BLE_*
    Examples/MAX32655/Bootloader
    Libraries/libs.mk
    Libraries/Cordio/*
    Libraries/CMSIS/Device/MCU1
    Libraries/PeriphDrivers
    Libraries/BlePhy
    Libraries/Boards
)

# Directories to ignore
declare -a ignored=(
    Libraries/Cordio/docs
    Libraries/Cordio/controller
)

# Reset shopt nullglob
$shopt                        # Revert nullglob

# Work starts here

# Create an associative array for ignored()
declare -A aignored=()
for ignore in "${ignored[@]}"; do aignored["${ignore%/}"]=1; done

# Copy the working directory list, skipping entries in ignored()
declare -a results=()
for watch in "${watched[@]}"; do test -v aignored["${watch%/}"] || results+=("${watch%/}"); done

# Print the resulting array, separated by newlines, in sorted order
printf '%s\n' "${results[@]}" | sort

# Get the diff from main
CHANGE_FILES=$(git diff --ignore-submodules --name-only remotes/origin/main)

for watch_file in "${results[@]}"; do 
    if [[ "$CHANGE_FILES" == *"$watch_file"* ]]; then
    BLE_FILES_CHANGED=1
    RUN_TEST=1
    printf "Foun a file in $watch_file \r\n" 
    fi
done