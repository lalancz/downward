#!/bin/bash

./build.py --debug

# Execute the fast-downward.py script
./fast-downward.py --debug --keep-sas misc/tests/benchmarks/miconic/s1-0.pddl --search "ibex(lmcut())"

# Write gdb commands to a file
echo "run --search \"ibex(lmcut())\" < output.sas" > gdb_commands.txt
echo "bt" >> gdb_commands.txt

# Run gdb with the commands from the file
gdb ./builds/debug/bin/downward -x gdb_commands.txt

# Clean up
rm gdb_commands.txt
