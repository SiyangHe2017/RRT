import matplotlib.pyplot as plt
import random
import math
import copy

show_animation = True


class Node(object):
    """
    RRT Node
    """

    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None  # !!! node.parent is just an index


class RRT(object):
    """
    Class for RRT Planning
    """

    def __init__(self, start, goal, obstacle_list, rand_area):
        """
        Setting Parameter

        start:Start Position [x,y]
        goal:Goal Position [x,y]
        obstacleList:obstacle Positions [[x,y,size],...]
        randArea:random sampling Area [min,max]

        """
        self.start = Node(start[0], start[1])
        self.end = Node(goal[0], goal[1])
        self.min_rand = rand_area[0]
        self.max_rand = rand_area[1]
        self.expandDis = 1  # This is used for checking goal
        self.goalSampleRate = 0.05  # Gaol bias is 0.05
        self.maxIter = 500  # what it is this?
        self.obstacleList = obstacle_list
        self.nodeList = [self.start]

    def random_node(self):
        """
        Creating random nodes
        :return:
        """
        node_x = random.uniform(self.min_rand, self.max_rand)
        node_y = random.uniform(self.min_rand, self.max_rand)
        node = [node_x, node_y]

        return node

    @staticmethod
    def get_nearest_list_index(node_list, rnd):
        """
        :param node_list:
        :param rnd:
        :return:
        """
        d_list = [(node.x - rnd[0]) ** 2 + (node.y - rnd[1]) ** 2 for node in node_list]
        min_index = d_list.index(min(d_list))
        return min_index

    @staticmethod
    def collision_check(new_node, obstacle_list):
        a = 1  # safe
        for (ox, oy, size) in obstacle_list:
            dx = ox - new_node.x
            dy = oy - new_node.y
            d = math.sqrt(dx * dx + dy * dy)
            if d <= size * 1.3:
                a = 0  # collision
        return a

    def get_distance_to_initial(self, temp_node):
        """
        Calculating the euclidean distance from the current node to its initial node
        :param temp_node: current node
        :return: distance: the distance to the origin from the current node along its father
        """
        distance = 0
        temp_node_index = self.nodeList.index(temp_node)
        while self.nodeList[temp_node_index].parent is not None:
            temp_node = self.nodeList[temp_node_index]
            node_parent_index = temp_node.parent
            node_parent = self.nodeList[node_parent_index]
            distance += math.sqrt((temp_node.x - node_parent.x) ** 2 + (temp_node.y - node_parent.y) ** 2)
            temp_node_index = node_parent_index
        return distance

    def Q_choose_parent(self, new_node):
        """
        For QRRT*
        :param new_node:
        :return:
        """
        return

    def choose_parent(self, new_node):
        """
        For RRT*
        Choosing a parent for the new node which makes the new node smallest cost to the root
        :param new_node:
        :return: minimu_index: -1 if there is no more nearby node
                                otherwise index of the node if there are nearby node
        """
        nearer_node_index_list = []
        for i in range(len(self.nodeList)):
            temp_node = self.nodeList[i]
            if ((temp_node.x - new_node.x) ** 2 + (temp_node.y - new_node.y) ** 2) < (1.5 * self.expandDis) ** 2:
                nearer_node_index_list.append(i)
        new_node_parent_node = self.nodeList[new_node.parent]
        minimum_distance = math.sqrt(
            (new_node_parent_node.x - new_node.x) ** 2 + (new_node_parent_node.y - new_node.y) ** 2)
        minimum_distance += self.get_distance_to_initial(new_node_parent_node)
        minimum_index = -1  # -1 stands for there is no node nearer than this
        for i in range(len(nearer_node_index_list)):
            temp_node_index = nearer_node_index_list[i]
            temp_node = self.nodeList[temp_node_index]
            temp_distance = math.sqrt((temp_node.x - new_node.x) ** 2 + (temp_node.y - new_node.y) ** 2)
            temp_distance += self.get_distance_to_initial(temp_node)
            if temp_distance < minimum_distance:
                minimum_distance = temp_distance
                minimum_index = temp_node_index
        return minimum_index

    def test_choose_parent(self, new_node):
        """
        This is used for testing the function choose_parent()
        """
        new_node_parent = self.choose_parent(new_node)
        if new_node_parent > -1:
            new_node_parent_node = self.nodeList[new_node_parent]
            distance = math.sqrt((new_node_parent_node.x - new_node.x)**2 + (new_node_parent_node.y - new_node.y)**2)
            print(distance, new_node_parent_node.x, new_node_parent_node.y, new_node.x, new_node.y)

    def rewire_rrt(self, new_node):
        """
        rewire the rrt path according to the rule of RRT*
        :param new_node: new_node
        :return: None
        """
        new_node_index = self.nodeList.index(new_node)
        new_node_distance_o = self.get_distance_to_initial(new_node)
        nearer_node_list = []
        for i in range(len(self.nodeList)):
            temp_node =self.nodeList[i]
            if ((temp_node.x - new_node.x) ** 2 + (temp_node.y - new_node.y) ** 2) < (1.5 * self.expandDis) ** 2:
                nearer_node_list.append(temp_node)
        if self.nodeList[new_node.parent] in nearer_node_list:
            nearer_node_list.remove(self.nodeList[new_node.parent])
        for i in range(len(nearer_node_list)):
            temp_node = nearer_node_list[i]
            temp_node_distance_o = self.get_distance_to_initial(temp_node)
            new_node_distance_temp_node = math.sqrt((new_node.x - temp_node.x)**2 + (new_node.y - temp_node.y)**2)
            if temp_node_distance_o > new_node_distance_o + new_node_distance_temp_node:
                temp_node.parent = new_node_index
        return

    def planning(self):
        """
        Path planning

        animation: flag for animation on or off
        """

        while True:
            # Random Sampling
            if random.random() > self.goalSampleRate:
                rnd = self.random_node()
            else:
                rnd = [self.end.x, self.end.y]

            # Find nearest node
            min_index = self.get_nearest_list_index(self.nodeList, rnd)
            # print(min_index)

            # expand tree
            nearest_node = self.nodeList[min_index]

            # 返回弧度制
            theta = math.atan2(rnd[1] - nearest_node.y, rnd[0] - nearest_node.x)

            new_node = copy.deepcopy(nearest_node)
            new_node.x += self.expandDis * math.cos(theta) * random.uniform(0, 2)
            new_node.y += self.expandDis * math.sin(theta) * random.uniform(0, 2)
            new_node.parent = min_index

            if not self.collision_check(new_node, self.obstacleList):
                continue

            test_new_node_parent_index = self.choose_parent(new_node)
            if test_new_node_parent_index > -1:
                new_node.parent = test_new_node_parent_index

            self.nodeList.append(new_node)

            self.rewire_rrt(new_node)

            # check goal
            dx = new_node.x - self.end.x
            dy = new_node.y - self.end.y
            d = math.sqrt(dx * dx + dy * dy)
            if d <= self.expandDis * 0.7:
                print("Goal!!")
                break

            if True:
                self.draw_graph(rnd)

        path = [[self.end.x, self.end.y]]
        last_index = len(self.nodeList) - 1
        while self.nodeList[last_index].parent is not None:
            node = self.nodeList[last_index]
            path.append([node.x, node.y])
            last_index = node.parent
        path.append([self.start.x, self.start.y])

        return path

    def draw_graph(self, rnd=None):
        """
        Draw Graph
        """
        print('aaa')
        plt.clf()  # 清除上次画的图
        if rnd is not None:
            plt.plot(rnd[0], rnd[1], "^g")
        for node in self.nodeList:
            if node.parent is not None:
                plt.plot([node.x, self.nodeList[node.parent].x], [
                    node.y, self.nodeList[node.parent].y], "-g")

        for (ox, oy, size) in self.obstacleList:
            # plt.plot(ox, oy, "sk", ms=10*size)
            temple_circle = plt.Circle((ox, oy), size, color='black')
            plt.gcf().gca().add_artist(temple_circle)

        plt.plot(self.start.x, self.start.y, "^r")
        plt.plot(self.end.x, self.end.y, "^b")
        plt.axis([self.min_rand, self.max_rand, self.min_rand, self.max_rand])
        plt.grid(True)
        plt.gca().set_aspect("equal")
        plt.pause(0.01)

    def draw_temp_static(self, path):
        """
        画出静态图像
        :return:
        """
        plt.clf()  # 清除上次画的图

        for node in self.nodeList:
            if node.parent is not None:
                plt.plot([node.x, self.nodeList[node.parent].x], [
                    node.y, self.nodeList[node.parent].y], "-g")

        for (ox, oy, size) in self.obstacleList:
            # plt.plot(ox, oy, "sk", ms=10*size)
            temple_circle = plt.Circle((ox, oy), size, color='black')
            plt.gcf().gca().add_artist(temple_circle)

        plt.plot(self.start.x, self.start.y, "^r")
        plt.plot(self.end.x, self.end.y, "^b")
        plt.axis([self.min_rand, self.max_rand, self.min_rand, self.max_rand])

        plt.plot([data[0] for data in path], [data[1] for data in path], '-r')
        plt.grid(True)
        plt.gca().set_aspect("equal")
        plt.pause(1)

    def draw_static(self, path):
        """
        画出静态图像
        :return:
        """
        plt.clf()  # 清除上次画的图

        for node in self.nodeList:
            if node.parent is not None:
                plt.plot([node.x, self.nodeList[node.parent].x], [
                    node.y, self.nodeList[node.parent].y], "-g")

        for (ox, oy, size) in self.obstacleList:
            # plt.plot(ox, oy, "sk", ms=10*size)
            temple_circle = plt.Circle((ox, oy), size, color='black')
            plt.gcf().gca().add_artist(temple_circle)

        plt.plot(self.start.x, self.start.y, "^r")
        plt.plot(self.end.x, self.end.y, "^b")
        plt.axis([self.min_rand, self.max_rand, self.min_rand, self.max_rand])

        plt.plot([data[0] for data in path], [data[1] for data in path], '-r')
        plt.grid(True)
        plt.gca().set_aspect("equal")
        plt.show()


def main():
    print("start RRT path planning")

    obstacle_list = [
        (3, 3, 1),
        (8, 2, 1),
        (2, 8, 1),
        (6, 6, 1)
        ]

    # Set Initial parameters
    rrt = RRT(start=[0, 0], goal=[8, 9], rand_area=[-2, 10], obstacle_list=obstacle_list)
    '''
    for i in range(1, len(path)):
        print(path[i])
    '''
    path = rrt.planning()
    rrt.draw_temp_static(path)

    iterator = 1
    while iterator < 2:
        iterator += 1
        path = rrt.planning()
        rrt.draw_temp_static(path)

    # Draw final path
    if show_animation:
        plt.close()
        rrt.draw_static(path)


if __name__ == '__main__':
    main()
