import torch
import torch.optim as optim


def J(x, y):
    return x**2 + y**2 + 1


x = torch.tensor(3.0, requires_grad=True)
y = torch.tensor(4.0, requires_grad=True)
optimizer = optim.Adam([x, y], lr=0.3)
while True:
    print(x, y)
    func = J(x.float(), y.float())
    print(func)

    optimizer.zero_grad()
    func.backward()
    optimizer.step()
    print(1111)
