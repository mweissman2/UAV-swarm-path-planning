import numpy as np

# Define the input x, weight matrices W and V
x = np.array([1, 2, 1])
W = np.array([[1, 0, 1], [1, -1, 0]])
V = np.array([0, 1])

# Define the activation function
def sigma(z):
    return np.maximum(0, z)

# Compute the hidden layer activations
h = sigma(np.dot(W, x))

# Compute the output y_hat
y_hat = sigma(np.dot(V, h))

# Compute the loss l
y = 1  # Specify the target value for y
loss = 0.5 * (y_hat - y) ** 2

print("y_hat:", y_hat)
print("loss:", loss)

