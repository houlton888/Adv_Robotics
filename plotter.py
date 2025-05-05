import matplotlib.pyplot as plt

# === Sample data ===
ground_truth = [(-33, -38), (94, 21), (64, 32)]
fk_estimates = [(85, -106), (-21, 14), (33, -80)]
kalman_estimates = [(-37,-40), (74, 21), (72, 32)]

# === Plot ===
plt.figure(figsize=(8, 8))

# FK points
for i, (gt, est) in enumerate(zip(ground_truth, fk_estimates), 1):
    plt.scatter(*gt, color='blue', marker='o', label='Ground Truth' if i == 1 else "")
    plt.scatter(*est, color='green', marker='x', label='FK Estimate' if i == 1 else "")
    plt.text(gt[0], gt[1], str(i), color='blue', fontsize=10)
    plt.text(est[0], est[1], str(i), color='green', fontsize=10)

# Kalman points
for i, (gt, est) in enumerate(zip(ground_truth, kalman_estimates), 1):
    plt.scatter(*est, color='red', marker='x', label='Kalman Estimate' if i == 1 else "")
    plt.text(est[0], est[1], str(i), color='red', fontsize=10)

# Labels and style
plt.xlabel("X position (cm)")
plt.ylabel("Y position (cm)")
plt.title("FK vs Kalman Estimates vs Ground Truth")
plt.grid(True)
plt.axis("equal")
plt.legend()
plt.tight_layout()
plt.show()
