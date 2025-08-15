import matplotlib.pyplot as plt

error_values = [-42.0, -41.0, -41.0, -40.0, -35.0, -23.0, -11.0, 1.0, 16.0, 23.0, 25.0, 21.0, 16.0, 9.0, 7.0, 2.0, 0.0, -1.0, 0.0, -2.0]
x_distance = [i * 10 for i in range(len(error_values))]

plt.figure(figsize=(10, 5))
plt.plot(x_distance, error_values, marker='o', linestyle='-', color='blue', label='Tracking Error')
plt.axhline(0, color='gray', linestyle='--', linewidth=1)  # Optional: zero-error reference line
plt.title("Line Tracking Error vs Distance Travelled")
plt.xlabel("Distance Travelled (mm)")
plt.ylabel("Error")
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()