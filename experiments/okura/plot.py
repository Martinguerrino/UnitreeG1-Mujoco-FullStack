"""Plot recorded trajectories using NumPy and Matplotlib."""

import argparse
from pathlib import Path

import numpy as np

def plot(directories, output):
    import matplotlib

    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(len(directories), 3, figsize=(13, 3 * len(directories)), squeeze=False)
    for row, directory in enumerate(directories):
        with np.load(directory / 'trajectory.npz') as data:
            t = data['time']
            axes[row, 0].plot(t, 1000 * (data['box'][:, 2] - data['box'][0, 2]))
            axes[row, 0].axhline(50, linestyle='--', color='gray')
            axes[row, 0].set_ylabel(directory.name.replace('okura_eval_20261001_', '') + '\nlift [mm]')
            axes[row, 1].plot(t, data['gripper_target'][:, 1], label='command')
            axes[row, 1].plot(t, data['gripper'][:, 1], label='measured')
            axes[row, 1].set_ylabel('right gripper [motor rad]')
            axes[row, 1].legend()
            axes[row, 2].plot(t, data['finger_contacts'], label='finger contacts')
            axes[row, 2].plot(t, data['bilateral'].astype(int), label='same-hand pair')
            axes[row, 2].set_ylabel('contacts')
            axes[row, 2].legend()
        for ax in axes[row]:
            ax.set_xlabel('simulation time [s]')
            ax.grid(alpha=0.2)
    fig.tight_layout()
    fig.savefig(output, dpi=150)
    plt.close(fig)



def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directories", nargs="+", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    plot(args.directories, args.output)


if __name__ == "__main__":
    main()
