#!/bin/bash

# for a in 25 30 35 40 45 50 55 60 70; do


for a in 25; do
  for b in 42; do
 	for c in {1..10}; do
	    sbatch --job-name="C${a}S${b}" Ruche/bap.sh "$a" "$b" "$c"
	done
  done
done
