import os
import re
import argparse
import matplotlib.pyplot as plt
import pandas as pd
import locale

text_scale = 1.25

# ---- CONFIGURACIÓN GLOBAL DE FUENTES ----
plt.rcParams.update({
    'font.size': 12*text_scale,          # Tamaño base para todo el texto
    'axes.titlesize': 16*text_scale,     # Tamaño del título
    'axes.labelsize': 14*text_scale,     # Tamaño de las etiquetas de los ejes (X e Y)
    'xtick.labelsize': 12*text_scale,    # Tamaño de los números en el eje X
    'ytick.labelsize': 12*text_scale,    # Tamaño de los números en el eje Y
    'legend.fontsize': 12*text_scale,    # Tamaño del texto de la leyenda
    'legend.title_fontsize': 13*text_scale # Tamaño del título de la leyenda
})

# ==========================================
# Parsear fichero de salida
# ==========================================

def parse_file(filepath):
    with open(filepath, 'r') as f:
        content = f.read()

    match = re.search(r'Mean step time:\s+([0-9,.]+) ns', content)

    if not match:
        return None

    mean_time = locale.atoi(match.group(1))
    return mean_time


# ==========================================
# Extraer parametros del nombre del fichero
# ==========================================

def parse_filename(filename):
    # scene10_b10000_t2_s600.txt
    pattern = r'scene(\d+)_b(\d+)_t(\d+)'

    match = re.search(pattern, filename)

    if not match:
        return None

    return {
        'scene': int(match.group(1)),
        'bodies': int(match.group(2)),
        'threads': int(match.group(3)),
    }


# ==========================================
# Leer directorio completo
# ==========================================

def load_results(results_dir):
    rows = []

    for file in os.listdir(results_dir):

        if not file.endswith('.txt'):
            continue

        meta = parse_filename(file)

        if meta is None:
            continue

        filepath = os.path.join(results_dir, file)

        mean_time = parse_file(filepath)

        if mean_time is None:
            continue

        row = {
            **meta,
            'mean_time_ns': mean_time
        }

        rows.append(row)

    return pd.DataFrame(rows)


# ==========================================
# Main
# ==========================================

def main():

    locale.setlocale(locale.LC_ALL, 'en_US.UTF-8')

    parser = argparse.ArgumentParser()

    parser.add_argument(
        '--scene',
        type=int,
        required=True,
        help='Escena a representar'
    )

    parser.add_argument(
        'input_dirs',
        nargs='+',
        help='Lista de carpetas de entrada'
    )

    args = parser.parse_args()

    # ==========================================
    # Configuración de la cuadrícula (Máximo 2 columnas)
    # ==========================================
    num_dirs = len(args.input_dirs)
    ncols = 2 if num_dirs > 1 else 1
    nrows = (num_dirs + ncols - 1) // ncols  # División entera superior

    # Crear figura con filas y columnas dinámicas
    fig, axes = plt.subplots(
        nrows,
        ncols,
        figsize=(6 * ncols, 5 * nrows), # Mantiene la proporción de 6x5 por gráfica
        sharex=True,
        sharey=True
    )

    # Convertir 'axes' en una lista/array plano para iterar fácilmente
    if num_dirs == 1:
        axes = [axes]
    else:
        axes = axes.flatten()

    # ==========================================
    # Dibujar cada benchmark
    # ==========================================

    for ax, directory in zip(axes, args.input_dirs):

        if not os.path.exists(directory):
            print(f'[WARNING] No existe: {directory}')
            continue

        df = load_results(directory)

        if df.empty:
            print(f'[WARNING] Sin datos en: {directory}')
            continue

        # Filtrar escena
        df = df[df['scene'] == args.scene]

        if df.empty:
            print(f'[WARNING] No hay datos para escena {args.scene} en {directory}')
            continue

        # Agrupar por número de hilos (threads)
        for threads in sorted(df['threads'].unique()):

            sub = df[df['threads'] == threads]
            sub = sub.sort_values('bodies')

            ax.plot(
                sub['mean_time_ns'] / 1e6,
                sub['bodies'],
                marker='o',
                label=f'{threads} hilos'
            )

        ax.set_title(directory)
        ax.grid(True)
        ax.legend()

    # Ocultar los subplots que sobren si el número de carpetas es impar
    for i in range(num_dirs, len(axes)):
        axes[i].axis('off')

    fig.supxlabel('Duración media del ciclo de simulación (ms)', fontsize=14*text_scale)
    fig.supylabel('Número de cuerpos', fontsize=14*text_scale)
    fig.suptitle(f'Comparación Benchmarks - Escena {args.scene}')

    plt.tight_layout()
    plt.show()


if __name__ == '__main__':
    main()