#!/usr/bin/python3

with open("../stdio.log") as f:
    insideCut = False
    for line in f:
        line = line.rstrip()
        if '--------------- CUT HERE -----------------' in line:
            insideCut = not insideCut
            continue
        if insideCut:
           print(line)
