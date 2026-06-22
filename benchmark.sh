#!/bin/bash
#SBATCH --job-name=physics_bench
#SBATCH --output=slurm-%j.out
#SBATCH --error=slurm-%j.err
#SBATCH --ntasks=1

# ============================
# Uso:
# sbatch benchmark.sh <programa> "<escenas>" "<cuerpos>" <steps> "<threads>" <timeout> <outdir>
#
# Ejemplo:
# sbatch benchmark.sh build/sample/sample "10 11 12" "128 256 512 1024 2048 4096 8192 16384" 600 "1 2 4 8 16 32" 20 results
# ============================

PROGRAM=$1
SCENES=($2)
BODIES=($3)
STEPS=$4
THREADS_LIST=($5)
TIMEOUT=$6
OUTDIR=$7

# Carpeta de resultados
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
  for threads in "${THREADS_LIST[@]}"; do

    if [ "$threads" -gt "$(nproc --all)" ]; then
      echo "Superado los hilos disponibles en el nodo ($(nproc --all))!"
      break 
    fi

    for bodies in "${BODIES[@]}"; do

      export OMP_NUM_THREADS=$threads

      BASENAME="scene${scene}_b${bodies}_t${threads}"
      OUTFILE="${OUTDIR}/${BASENAME}.txt"

      echo "Ejecutando: d=$scene b=$bodies t=$threads"

      timeout "$TIMEOUT" "$PROGRAM" -e -d "$scene" -b "$bodies" -s "$STEPS" > "$OUTFILE" 2> /dev/null
      status=$?

      if [ $status -eq 124 ]; then
        echo "TIMEOUT detectado en b=$bodies con t=$threads!"
        break 
      fi
    done
  done
done

echo "Todas las ejecuciones finalizadas."
