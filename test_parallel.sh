#!/bin/bash
#SBATCH --job-name=physics_bench
#SBATCH --output=slurm-%j.out
#SBATCH --error=slurm-%j.err
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=64
#SBATCH --time=02:00:00

# ============================
# Uso:
# sbatch script.sh <programa> "<escenas>" "<cuerpos>" <steps> "<threads>"
#
# Ejemplo:
# sbatch script.sh ./test_parallel.sh "1 2 3" "100 500" 1000 "1 2 4 8 16"
# ============================

PROGRAM=$1
SCENES=($2)
BODIES=($3)
STEPS=$4
THREADS_LIST=($5)

# Carpeta de resultados
OUTDIR="results_${SLURM_JOB_ID}"
mkdir -p "$OUTDIR"

echo "Programa: $PROGRAM"
echo "Escenas: ${SCENES[@]}"
echo "Cuerpos: ${BODIES[@]}"
echo "Steps: $STEPS"
echo "Threads: ${THREADS_LIST[@]}"
echo "Resultados en: $OUTDIR"

# ============================
# Loop de combinaciones
# ============================

for scene in "${SCENES[@]}"; do
  for bodies in "${BODIES[@]}"; do
    for threads in "${THREADS_LIST[@]}"; do

      export OMP_NUM_THREADS=$threads

      BASENAME="scene${scene}_b${bodies}_t${threads}_s${STEPS}"
      OUTFILE="${OUTDIR}/${BASENAME}.txt"
      PERF_DATA="${OUTDIR}/${BASENAME}.perf.data"

      echo "Ejecutando: d=$scene b=$bodies t=$threads s=$STEPS"

      # Ejecutar con perf
      perf record -g -o "$PERF_DATA" \
        "$PROGRAM" -d "$scene" -b "$bodies" -s "$STEPS" \
        > "$OUTFILE" 2> /dev/null

    done
  done
done

echo "Todas las ejecuciones finalizadas."
