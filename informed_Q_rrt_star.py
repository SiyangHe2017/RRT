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
        self.c_best = 1000000000  # stands for positive infinity

    def random_node(self):
        """
        Creating random node
        :return:
        """
        node_x = random.uniform(self.min_rand, self.max_rand)
        node_y = random.uniform(self.min_rand, self.max_rand)
        node = [node_x, node_y]

        return node

    @staticmethod
    def get_nearest_list_index(node_list, rnd):
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
            if d <= size:
                a = 0  # collision
        return a

    @staticmethod
    def calculate_path_distance(path):
        distance = 0
        for i in range(len(path) - 1):
            d_x = path[i][0] - path[i + 1][0]
            d_y = path[i][1] - path[i + 1][1]
            distance += math.sqrt(d_x * d_x + d_y * d_y)
        return distance

    @staticmethod
    def node_collision_circle_free(x1, y1, x2, y2, x0, y0, r):
        k = (y1 - y2) / (x1 - x2)
        d0 = abs(k * x0 - y0 + y1 - k * x1) / math.sqrt(k * k + 1)
        if d0 <= r:
            return False
        else:
            return True

    def node_collision_block_free(self, node_1, node_2):
        x1 = node_1.x
        y1 = node_1.y
        x2 = node_2.x
        y2 = node_2.y
        for (x0, y0, r) in self.obstacleList:
            if not self.node_collision_circle_free(x1, y1, x2, y2, x0, y0, r * 1.1):
                return False
        return True

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
        For QRRT star
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
        minimum_index = -1 
        for i in range(len(nearer_node_index_list)):
            temp_node_index = nearer_node_index_list[i]
            temp_node = self.nodeList[temp_node_index]
            temp_distance = math.sqrt((temp_node.x - new_node.x) ** 2 + (temp_node.y - new_node.y) ** 2)
            temp_distance += self.get_distance_to_initial(temp_node)
            if temp_distance < minimum_distance:
                if self.node_collision_block_free(new_node, temp_node):
                    minimum_distance = temp_distance
                    minimum_index = temp_node_index
            while temp_node.parent is not None:
                temp_node_index = temp_node.parent
                temp_node = self.nodeList[temp_node_index]
                temp_distance = math.sqrt((temp_node.x - new_node.x) ** 2 + (temp_node.y - new_node.y) ** 2)
                temp_distance += self.get_distance_to_initial(temp_node)
                if temp_distance < minimum_distance:
                    if self.node_collision_block_free(new_node, temp_node):
                        minimum_distance = temp_distance
                        minimum_index = temp_node_index
        return minimum_index

    def Q_rewire_rrt(self, new_node):
        """
        rewire the rrt path according to the rule of Q-RRT*
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
            shortest_distance = self.get_distance_to_initial(temp_node)
            new_node_distance_temp_node = math.sqrt((new_node.x - temp_node.x)**2 + (new_node.y - temp_node.y)**2)
            if shortest_distance > new_node_distance_o + new_node_distance_temp_node:
                if self.node_collision_block_free(temp_node, new_node):
                    temp_node.parent = new_node_index
                    shortest_distance = new_node_distance_o + new_node_distance_temp_node
            
            temp_new_node_index = new_node_index
            while(self.nodeList[temp_new_node_index].parent is not None):
                temp_new_node_index = self.nodeList[temp_new_node_index].parent
                temp_new_node = self.nodeList[temp_new_node_index]
                temp_new_node_distance_o = self.get_distance_to_initial(temp_new_node)
                temp_new_node_distance_temp_node = math.sqrt((temp_new_node.x - temp_node.x)**2 + (temp_new_node.y - temp_node.y)**2)
                if shortest_distance > temp_new_node_distance_o + temp_new_node_distance_temp_node:
                    if self.node_collision_block_free(temp_node, temp_new_node):
                        temp_node.parent = temp_new_node_index
                        shortest_distance = temp_new_node_distance_o + temp_new_node_distance_temp_node
        return

    def is_node_rejection(self, new_node):
        """
        For informed-RRT star
        test if it is in the eclipse area or not
        :param new_node:
        :return: True if it is out of the eclipse, False if it is in the eclipse
        """
        d0_x = new_node.x - self.start.x
        d0_y = new_node.y - self.start.y
        d0 = math.sqrt(d0_x * d0_x + d0_y * d0_y)
        d1_x = new_node.x - self.end.x
        d1_y = new_node.y - self.end.y
        d1 = math.sqrt(d1_x * d1_x + d1_y * d1_y)
        if d0 + d1 > self.c_best:
            return True
        else:
            return False

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

            if not self.node_collision_block_free(new_node, nearest_node):
                continue

            if self.is_node_rejection(new_node):
                continue

            test_new_node_parent_index = self.Q_choose_parent(new_node)
            if test_new_node_parent_index > -1:
                new_node.parent = test_new_node_parent_index

            self.nodeList.append(new_node)

            self.Q_rewire_rrt(new_node)

            # check goal
            dx = new_node.x - self.end.x
            dy = new_node.y - self.end.y
            d = math.sqrt(dx * dx + dy * dy)
            if d <= self.expandDis * 0.7:
                print("Goal!!")
                break

            if True:
                # self.draw_graph(rnd)
                pass

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
        (2, 2, 1),
        (8, 2, 1),
        (2, 8, 1),
        (6, 6, 1)
    ]

    # Set Initial parameters
    rrt = RRT(start=[0, 0], goal=[8, 9], rand_area=[-5, 15], obstacle_list=obstacle_list)
    path = rrt.planning()
    rrt.draw_temp_static(path)

    rrt.c_best = rrt.calculate_path_distance(path)
    print(rrt.c_best)

    iterator = 1
    while iterator < 20:
        iterator += 1
        path = rrt.planning()
        rrt.draw_temp_static(path)
        if rrt.c_best > rrt.calculate_path_distance(path):
            rrt.c_best = rrt.calculate_path_distance(path)
        print(rrt.c_best)

    # Draw final path
    if show_animation:
        plt.close()
        rrt.draw_static(path)


if __name__ == '__main__':
    main()