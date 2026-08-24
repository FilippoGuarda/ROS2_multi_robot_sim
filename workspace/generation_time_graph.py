import numpy as np
from pathlib import Path
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors

out_dir = Path("workspace/output")
out_dir.mkdir(exist_ok=True)

# A star computation time approximation based on BFS paper
Astar =  lambda x: 0.03 * x/10000

def metric_map(x, y):
    return (y ** 2) * Astar(x)

def bfs_mas(x, y):

    # Topological graph computation time 
    T = (2.3*10**-5) * x + 0.026
    # Search space compression factor
    C = 97

    return  (y ** 2) * Astar(x) / C  + T 


def main(
    x_range=(1, 10000),
    nx=800,
    fleet_min=1,
    fleet_max=10,
    fleet_step=1,
    cmap="plasma",
    out_path= out_dir / "t_diff_heatmap.png",
):
    x = np.linspace(*x_range, nx)
    fleet_sizes = np.arange(fleet_min, fleet_max + 1, fleet_step)

    X, Y = np.meshgrid(x, fleet_sizes)

    Z_metric = metric_map(X, Y)
    Z_bfs = bfs_mas(X, Y)
    Z_diff = Z_metric - Z_bfs

    zmin, zmax = Z_diff.min(), Z_diff.max()
    norm = mcolors.Normalize(vmin=zmin, vmax=zmax) 

    fig, ax = plt.subplots(figsize=(11, 6))

    mesh = ax.pcolormesh(x, fleet_sizes, Z_diff, cmap=cmap, norm=norm, shading="nearest")

    if zmin <= 0 <= zmax:
        boundary = ax.contour(X, Y, Z_diff, levels=[0], colors="red", linewidths=2.2)
        ax.clabel(boundary, fmt="Metric = BFS-MAS", fontsize=9)

    cbar = fig.colorbar(mesh, ax=ax, pad=0.02)
    cbar.set_label(r"Path finding time difference")
    cbar.set_ticks(np.linspace(zmin, zmax, 8))

    ax.set_yticks(fleet_sizes)
    ax.set_xlabel("Map size in pixels")
    ax.set_ylabel("Robot fleet size")
    ax.set_title("Top-view heatmap of cost matrix generation time for a fleet of N robots" "\n" " on metric VS topological graph, including graph generation time")

    plt.tight_layout()
    plt.savefig(out_path, dpi=200)
    plt.close(fig)

    return X, Y, Z_diff


if __name__ == "__main__":
    main() 