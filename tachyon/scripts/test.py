import numpy as np
import matplotlib.pyplot as plt


def generated_steps_for_a_step(n_steps=10):
    x_values = []
    y_values = []
    r = 0.15
    theta = (np.pi / 6) * 5
    d = np.sqrt(np.power(r, 2) + np.power(r, 2) - (2 * np.power(r, 2) * np.cos(theta)))
    h = r * np.cos(theta / 2)
    b = r - (d / 2)

    for alpha in np.arange(((np.pi / 2) - (theta / 2)), 1.001 * ((np.pi / 2) + (theta / 2)), theta / n_steps):
        x = r - (r * np.cos(alpha))
        y = r * np.sin(alpha)

        x_values.append(x - b)
        y_values.append(y - h)

    return x_values, y_values


print(generated_steps_for_a_step(10))
steps = generated_steps_for_a_step()
plt.plot(steps[0], steps[1])
plt.axis([0, 0.3, 0, 0.3])
plt.show()
