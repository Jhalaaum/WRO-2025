import matplotlib.pyplot as plt

error_values = [-19.5, -19.5, -22.5, -21.5, -25.5, -27.5, -26.5, -18.5, -10.5, -6.5, -2.5, 2.5, 5.5, 8.5, 8.5, 8.5, 7.5, 6.5, 5.5, 6.5, 7.5, 6.5, 5.5, 3.5, 0.5, -1.5, -2.5, -4.5, -5.5, -5.5, -3.5, -2.5, -0.5, 0.5, 0.5, 1.5, 2.5, 2.5, 2.5, 2.5, 4.5, 2.5, -0.5, -2.5, -2.5, -3.5, -1.5, -1.5, -1.5, -0.5, 1.5, 2.5, 0.5, -1.5, 0.5, 0.5, 1.5, 4.5, 6.5, 7.5, 9.5, 10.5, 9.5, 6.5, 2.5, -1.5, -6.5, -11.5, -12.5, -7.5, 40.5, 42.5, 4.5, -30.5, -33.5, -33.5, -28.5, -14.5, -9.5, -1.5, -0.5, -0.5, 1.5, 7.5, 11.5, 11.5, 7.5, 6.5, 6.5, 4.5, 3.5, 3.5, 0.5, 0.5, -0.5, -1.5, -2.5, -3.5, -4.5]
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