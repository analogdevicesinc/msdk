#! /usr/bin/env python3

################################################################################
# Copyright (C) 2020 Maxim Integrated Products, Inc., All Rights Reserved.
#
# Permission is hereby granted, free of charge, to any person obtaining a
# copy of this software and associated documentation files (the "Software"),
# to deal in the Software without restriction, including without limitation
# the rights to use, copy, modify, merge, publish, distribute, sublicense,
# and/or sell copies of the Software, and to permit persons to whom the
# Software is furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included
# in all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
# OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
# MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
# IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
# OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
# ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
# OTHER DEALINGS IN THE SOFTWARE.
#
# Except as contained in this notice, the name of Maxim Integrated
# Products, Inc. shall not be used except as stated in the Maxim Integrated
# Products, Inc. Branding Policy.
#
# The mere transfer of this software does not imply any licenses
# of trade secrets, proprietary technology, copyrights, patents,
# trademarks, maskwork rights, or any other form of intellectual
# property whatsoever. Maxim Integrated Products, Inc. retains all
# ownership rights.
#
###############################################################################

# plot_per_results.py
#
# Create plots of the CI PER test results
#

import sys
import argparse
from argparse import RawTextHelpFormatter
from time import sleep
import matplotlib.pyplot as plt
import numpy as np
import os
import pandas
import itertools

RES_DIR = '/home/btm-ci/Workspace/ci_results/per'

SPEC = 30  # per spec in %
phy_str = ["", "1M", "2M", "S8", "S2"]

# Setup the command line description text
descText = """
PER plotting tools.

Takes .csv file arguments and creates PDF plots.

First row in each file should be a header row with the following columns:
packetLen,phy,atten,txPower,perMaster,perSlave
"""

# Parse the command line arguments
parser = argparse.ArgumentParser(
    description=descText, formatter_class=RawTextHelpFormatter)
parser.add_argument('csvFile', help='csv file containing PER data.')
parser.add_argument('desc', help='Description of data.')
parser.add_argument('basename', help='PDF file base name.')

args = parser.parse_args()

print("csvFile :", args.csvFile)

csv_full_path = f'{args.csvFile}'
csv_full_path = os.path.expanduser(csv_full_path)
print(f'csv full: {csv_full_path}')

pdf_file_name = args.csvFile.replace('.csv', '.pdf')
pdf_file_name = os.path.expanduser(pdf_file_name)

# Create the plots
phys = []
lens = []
attens = []
txPowers = []

df = pandas.read_csv(csv_full_path)
lens = df["packetLen"].unique()
phys = df["phy"].unique()
attens = df["atten"].unique()
txPowers = df["txPower"].unique()

print("lens     :", lens)
print("phys     :", phys)
print("attens   :", attens)
print("txPowers :", txPowers)

print("--------------------------------------------------------------------------------------------")
row = len(lens) * len(txPowers)
col = len(phys)
print(f'row: {row}, col: {col}')

if row > 1 or col > 1:
    fig, axs = plt.subplots(row, col)

    name = pdf_file_name.split('/')[-1]
    board = name.split('_')[2].replace('.pdf', '').upper()
    fig.suptitle(f'Packet Error Rate vs Attenuation\n{board}', fontsize=10)

    if axs.ndim == 1:
        plt.subplots_adjust(top=0.83, hspace=0.5)
    else:
        fig.tight_layout()
        plt.subplots_adjust(top=0.80, bottom=0.1, hspace=0.5)

    case = 0
    for packetLen, phy, txPower in itertools.product(lens, phys, txPowers):
        print(f'CASE: {case + 1}')
        col = case % len(phys)
        row = int(case / len(phys))
        print(f'row: {row}, col: {col}')

        # Create line plot with atten to perSlave
        print("len     :", packetLen)
        print("phy     :", phy)
        print("txPower :", txPower)
        tempDf = df.loc[(df['packetLen'] == packetLen) & (
            df['phy'] == phy) & (df['txPower'] == txPower)]
        '''
        name = pdf_file_name.split('/')[-1]
        board = name.split('_')[2].replace('.pdf', '').upper()
        fig.suptitle(f'Packet Error Rate vs Attenuation\n{board}', fontsize=10)
        fig.tight_layout()
        plt.subplots_adjust(bottom=0.1)
        '''
        title = f'packet len: {packetLen}\nphy: {phy_str[phy]}\ntxPower:{txPower}'
        if axs.ndim == 1:
            axs[row].set_title(title, fontdict={'fontsize': 6, 'fontweight': 'medium'})
            axs[row].set_xlabel('Attenuation, dBm', fontdict={"fontsize": 5})
            axs[row].set_ylabel('PER (%)', fontdict={"fontsize": 5})
            axs[row].tick_params(axis='both', which='major', labelsize=4)
            axs[row].plot(tempDf["atten"], tempDf["perSlave"], "-x", linewidth=0.25, ms=0.5)
            axs[row].axhline(y=SPEC, color='r', linestyle=':', linewidth=0.5)
        else:
            axs[row, col].set_title(title, fontdict={'fontsize': 6, 'fontweight': 'medium'})
            axs[row, col].set_xlabel('Attenuation, dBm', fontdict={"fontsize": 5})
            axs[row, col].set_ylabel('PER (%)', fontdict={"fontsize": 5})
            axs[row, col].tick_params(axis='both', which='major', labelsize=4)
            axs[row, col].plot(tempDf["atten"], tempDf["perSlave"], "-x", linewidth=0.25, ms=0.5)
            axs[row, col].axhline(y=SPEC, color='r', linestyle=':', linewidth=0.5)

        a = list(tempDf['atten'])
        p = list(tempDf['perSlave'])
        for i in range(len(a)):
            if p[i] > SPEC:
                print(f'{a[i]}, {p[i]}')
                if axs.ndim == 1:
                    axs[row].axvline(x=a[i], color='r', linestyle=':', linewidth=0.5)
                    axs[row].text(a[i], p[i], f'  {p[i]}% @ {a[i]} dBm', horizontalalignment='left',
                                  verticalalignment='center', fontsize=3)
                else:
                    axs[row, col].axvline(x=a[i], color='r', linestyle=':', linewidth=0.5)
                    axs[row, col].text(a[i], p[i], f'  {p[i]}% @ {a[i]} dBm', horizontalalignment='left',
                                       verticalalignment='center', fontsize=3)
                break

        # note
        fig.text(.5, .01, "Run on all data channels (no advertising channels).", ha='center',
                 fontdict={"fontsize": 5})

        print()
        case += 1

    saved_file = pdf_file_name
    print(saved_file)

    plt.savefig(saved_file)

    # save to a png file
    png_file = pdf_file_name.replace(".pdf", ".png")
    print(f'Save to file: {png_file}.')
    plt.savefig(png_file)
    # plt.show()

# -------------------------------------------------------------------------------------------------
# Save each test into separated PDF files.
# -------------------------------------------------------------------------------------------------
case = 0
for packetLen, phy, txPower in itertools.product(lens, phys, txPowers):
    print(f'CASE: {case + 1}')

    # Create line plot with atten to perSlave
    print("len     :", packetLen)
    print("phy     :", phy)
    print("txPower :", txPower)
    tempDf = df.loc[(df['packetLen'] == packetLen) & (
        df['phy'] == phy) & (df['txPower'] == txPower)]

    name = pdf_file_name.split('/')[-1]
    board = name.split('_')[2].replace('.pdf', '').upper()

    fig = plt.figure()
    ax1 = fig.add_axes((0.1, 0.2, 0.8, 0.7))

    title = f'Packet Error Rate vs Attenuation\n'\
            f'\n{board}\n'\
            f'Packet length: {packetLen}, PHY: {phy_str[phy]}, txPower:{txPower}'
    ax1.set_title(title)
    ax1.set_xlabel('Attenuation, dBm')
    ax1.set_ylabel('PER, Percentage')

    ax1.plot(tempDf["atten"], tempDf["perSlave"], "-x", linewidth=1, ms=2)

    a = list(tempDf['atten'])
    p = list(tempDf['perSlave'])
    plt.axhline(y=SPEC, color='r', linestyle=':', linewidth=0.75)
    for i in range(len(a)):
        if p[i] > SPEC:
            print(f'{a[i]}, {p[i]}')
            plt.axvline(x=a[i], color='r', linestyle=':', linewidth=0.75)
            plt.text(a[i], p[i], f'  {p[i]}% @ {a[i]} dBm', horizontalalignment='left',
                     verticalalignment='center', fontsize=8)
            break

    # resize the figure to match the aspect ratio of the Axes
    fig.set_size_inches(7, 8, forward=True)
    #fig.text(.5, .10, args.desc, ha='center', fontdict={"fontsize": 12})
    fig.text(.5, .05, "Run on all data channels (no advertising channels).", ha='center',
             fontdict={"fontsize": 12})

    filename = csv_full_path.replace('.csv', '')
    filename += f'_{packetLen}_{phy_str[phy]}_{txPower}.pdf'
    print(f'Save to file: {filename}.')
    plt.savefig(filename)

    # save to a png file
    png_file = filename.replace(".pdf", ".png")
    print(f'Save to file: {png_file}.')
    plt.savefig(png_file)

    print()
    case += 1

print("DONE!")

sys.exit(0)
