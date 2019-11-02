import random
class test():
    def __init__(self):
        self.list = []

    def hello(self):
        self.list.append(2)

a =test()
a.hello()
a.hello()
print(a.list)
for i in range(5):
    print(random.uniform(0, 1))