#!/bin/bash
# Submission script
#SBATCH --job-name=coman_matthew_opti
#SBATCH --time=20:00:00 # hh:mm:ss
#SBATCH --output=run.txt
#
#SBATCH --ntasks=51 # number of agents: 1 master + n-1 slaves
#SBATCH --mem-per-cpu=500 # megabytes
#
#SBATCH --mail-user=b.somers@student.uclouvain.be
#SBATCH --mail-type=ALL

# run the executable
mpirun ./dispatcher_opti
