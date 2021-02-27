#!/bin/bash

# Clean and create the sim port files so that Porpoise and the sim can communicate with each other
rm /tmp/bb_to_sim.txt
rm /tmp/sim_to_bb.txt
touch /tmp/bb_to_sim.txt
touch /tmp/sim_to_bb.txt