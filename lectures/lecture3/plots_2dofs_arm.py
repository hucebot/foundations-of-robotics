import numpy as np
import matplotlib.pyplot as plt
import glob
import os

def load_logs(pattern="gradient_descent_2dofs_arm_log_*.csv"):
    files = glob.glob(pattern)
    if not files:
        print(f"No log files found for pattern {pattern}!")
        return {}

    sigma_groups = {}   # {sigma_value: [list of runs]}

    for f in files:
        data = np.loadtxt(f, delimiter=",", skiprows=1)
        q0 = data[:, 0]
        q1 = data[:, 1]
        x = data[:, 2]
        y = data[:, 3]
        sigma = float(data[0, 4])     # sigma is constant per file
        cost = data[:, 5]

        if sigma not in sigma_groups:
            sigma_groups[sigma] = []

        sigma_groups[sigma].append({
            "file": os.path.basename(f),
            "q0": q0,
            "q1": q1,
            "x": x,
            "y": y,
            "cost": cost
        })

    return sigma_groups


def plot_groups(groups, gain_label="σ", plot_show=True):
    # --- Plot joint trajectories ---
    plt.figure("Joint Values")
    for sigma, runs in groups.items():
        for r in runs:
            t = np.arange(len(r["q0"]))
            plt.plot(t, r["q0"], label=f"q0 {gain_label}={sigma}")
            plt.plot(t, r["q1"], label=f"q1 {gain_label}={sigma}")

    plt.xlabel("Iteration")
    plt.ylabel("Joint Value [rad]")
    plt.title("Joint Evolution")
    plt.legend()
    plt.grid(True)
    plt.savefig("joint_trajectories.png", dpi=200)

    # --- Plot end-effector trajectory ---
    plt.figure("End-Effector Trajectories")
    for sigma, runs in groups.items():
        for r in runs:
            plt.plot(r["x"], r["y"], marker='o', markersize=3, label=f"{gain_label}={sigma}")

    plt.xlabel("x")
    plt.ylabel("y")
    plt.title("End-Effector Trajectories")
    plt.legend()
    plt.grid(True)
    plt.axis("equal")
    plt.savefig("ee_trajectories.png", dpi=200)

    # --- Plot cost vs iteration ---
    plt.figure("Cost")
    for sigma, runs in groups.items():
        for r in runs:
            t = np.arange(len(r["cost"]))
            plt.plot(t, r["cost"], label=f"{gain_label}={sigma}")

    plt.xlabel("Iteration")
    plt.ylabel("Cost")
    plt.title("Cost Decrease")
    plt.legend()
    plt.grid(True)
    plt.savefig("cost.png", dpi=200)

    if plot_show:
        plt.show()
        
def main():
    groupsA = load_logs()   # loads all your saved files
    groupsB = load_logs(pattern="clik_2dofs_arm_log_*.csv")
    if groupsA and groupsB:
        plot_groups(groupsA, plot_show=False)
        plot_groups(groupsB, gain_label="K", plot_show=True)
    elif groupsA:
        plot_groups(groupsA)
    elif groupsB:
        plot_groups(groupsB, gain_label="K", plot_show=True)
        
if __name__ == "__main__":
    main()
