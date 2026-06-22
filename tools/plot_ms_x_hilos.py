import os
import re
import argparse
import matplotlib.pyplot as plt
import pandas as pd
import locale

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

    # Crear figura
    fig, axes = plt.subplots(
        1,
        len(args.input_dirs),
        figsize=(6 * len(args.input_dirs), 5),
        sharey=True
    )

    # Cuando solo hay un subplot
    if len(args.input_dirs) == 1:
        axes = [axes]

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

        # Agrupar por número de cuerpos
        for bodies in sorted(df['bodies'].unique()):

            sub = df[df['bodies'] == bodies]
            sub = sub.sort_values('threads')

            ax.plot(
                sub['threads'],
                sub['mean_time_ns'] / 1e6,
                marker='o',
                label=f'{bodies} bodies'
            )

        ax.set_title(directory)
        ax.set_xlabel('Threads')
        ax.grid(True)
        ax.legend()

    axes[0].set_ylabel('Mean step time (ms)')

    fig.suptitle(f'Benchmark comparison - Scene {args.scene}')

    plt.tight_layout()
    plt.show()


if __name__ == '__main__':
    main()