#!/bin/bash

./build.py --debug

# Execute the fast-downward.py script
./fast-downward.py --debug misc/tests/benchmarks/miconic/s1-0.pddl --search "ibex(lmcut())"
