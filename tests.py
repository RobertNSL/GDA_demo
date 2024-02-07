import torch
import torch.optim as optim


def function_to_minimize(x, y):
    return (x ** 2) + 2*(y ** 2)


# Initialize a starting point for gradient descent
x = torch.tensor(17.0, requires_grad=True)
y = torch.tensor(1.0, requires_grad=True)

# Set hyperparameters for gradient descent
learning_rate = 1
num_iterations = 100

# Define the Adam optimizer
optimizer = optim.Adam([x, y], lr=learning_rate)

# Perform gradient descent
for i in range(num_iterations):
    # Forward pass: compute predicted y by passing x to the function
    y_pred = function_to_minimize(x, y)

    # Zero gradients, perform a backward pass, and update the weights.
    optimizer.zero_grad()
    y_pred.backward()
    optimizer.step()

# Print the maximum value found
print("Minimum value found:", function_to_minimize(x.item(), y.item()))
print("Value of x,y at minimum:", x.item(), y.item())
