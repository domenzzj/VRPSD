"""
作者：zhaozhijie
日期：2022年10月25日10时22分
"""
import math
import copy
import random
from tools import *


class ALNS:
    def __init__(self):
        self.rand_d_max = 0.4  # 随机破坏程度上限
        self.rand_d_min = 0.1  # 随机破坏程度下限
        self.worst_d_max = 20  # 最坏破坏程度上限
        self.worst_d_min = 5  # 最坏破坏程度下限
        self.regret_n = 5  # 次优位置个数
        self.r1 = 30  # 算子奖励1
        self.r2 = 18  # 算子奖励2
        self.r3 = 12  # 算子奖励3
        self.rho = 0.6  # 算子权重衰减系数
        self.d_weight = np.ones(2) * 10  # 破坏算子权重
        self.d_select = np.zeros(2)  # 破坏算子被选中次数/每轮
        self.d_score = np.zeros(2)  # 破坏算子被奖励得分/每轮
        self.d_history_select = np.zeros(2)  # 破坏算子历史共计被选中次数
        self.d_history_score = np.zeros(2)  # 破坏算子历史共计被奖励得分
        self.r_weight = np.ones(3) * 10  # 修复算子权重
        self.r_select = np.zeros(3)  # 修复算子被选中次数/每轮
        self.r_score = np.zeros(3)  # 修复算子被奖励得分/每轮
        self.r_history_select = np.zeros(3)  # 修复算子历史共计被选中次数
        self.r_history_score = np.zeros(3)  # 修复算子历史共计被奖励得分

    @staticmethod
    def initParam(model):
        for i in range(len(model.node_list)):
            for j in range(i + 1, len(model.node_list)):
                d = math.sqrt((model.node_list[i].x_coord - model.node_list[j].x_coord) ** 2 +
                              (model.node_list[i].y_coord - model.node_list[j].y_coord) ** 2)
                model.distance[i, j] = d
                model.distance[j, i] = d

    @staticmethod
    def genInitialSol(model):
        node_seq = copy.deepcopy(model.node_id_list)
        random.seed(random.randint(0, 10))
        random.shuffle(node_seq)
        return node_seq

    @staticmethod
    def splitRoutes(nodes_seq, model):
        num_vehicle = 0
        vehicle_routes = []
        route = []
        remained_cap = model.vehicle_cap
        for node_no in nodes_seq:
            if remained_cap - model.node_list[node_no].max_demand >= 0:
                route.append(node_no)
                remained_cap = remained_cap - model.node_list[node_no].max_demand
            else:
                vehicle_routes.append(route)
                route = [node_no]
                num_vehicle = num_vehicle + 1
                remained_cap = model.vehicle_cap - model.node_list[node_no].max_demand
        vehicle_routes.append(route)
        num_vehicle = num_vehicle + 1
        return num_vehicle, vehicle_routes

    @staticmethod
    def calDistance(route, model):
        distance = 0
        school = model.school
        for i in range(len(route) - 1):
            distance += model.distance[route[i], route[i + 1]]
        first_node = model.node_list[route[0]]
        last_node = model.node_list[route[-1]]
        distance += math.sqrt((school.x_coord - first_node.x_coord) ** 2 + (school.y_coord - first_node.y_coord) ** 2)
        distance += math.sqrt((school.x_coord - last_node.x_coord) ** 2 + (school.y_coord - last_node.y_coord) ** 2)
        return distance

    def calObj(self, nodes_seq, model):
        num_vehicle, vehicle_routes = self.splitRoutes(nodes_seq, model)
        distance = 0
        for route in vehicle_routes:
            distance += self.calDistance(route, model)

        return num_vehicle, vehicle_routes, distance

    def resetScore(self):
        self.d_select = np.zeros(2)
        self.d_score = np.zeros(2)

        self.r_select = np.zeros(3)
        self.r_score = np.zeros(3)

    def selectDestoryRepair(self):
        d_weight = self.d_weight
        d_cumsumprob = (d_weight / sum(d_weight)).cumsum()
        d_cumsumprob -= np.random.rand()
        destory_id = list(d_cumsumprob > 0).index(True)

        r_weight = self.r_weight
        r_cumsumprob = (r_weight / sum(r_weight)).cumsum()
        r_cumsumprob -= np.random.rand()
        repair_id = list(r_cumsumprob > 0).index(True)
        return destory_id, repair_id

    # 根据被选择的destory算子返回需要被移除的节点index序列。
    def doDestory(self, destory_id, model, sol):
        if destory_id == 0:
            reomve_list = self.createRandomDestory(model)
        else:
            reomve_list = self.createWorseDestory(model, sol)
        return reomve_list

    # 根据被选择的repair算子对当前接进行修复操作。
    def doRepair(self, repair_id, remove_list, model, sol):
        if repair_id == 0:
            new_sol = self.createRandomRepair(remove_list, model, sol)
        elif repair_id == 1:
            new_sol = self.createGreedyRepair(remove_list, model, sol)
        else:
            new_sol = self.createRegretRepair(remove_list, model, sol)
        return new_sol

    # 随机破坏算子
    def createRandomDestory(self, model):
        d = random.uniform(self.rand_d_min, self.rand_d_max)
        reomve_list = random.sample(range(len(model.node_list)), int(d * len(model.node_list)))
        return reomve_list

    # 贪婪破坏算子
    def createWorseDestory(self, model, sol):
        deta_f = []
        for node_no in sol.nodes_seq:
            nodes_seq_ = copy.deepcopy(sol.nodes_seq)
            nodes_seq_.remove(node_no)
            vehicle_num, vehicle_routes, distance = self.calObj(nodes_seq_, model)
            deta_f.append(sol.vehicle_num - vehicle_num)
        sorted_id = sorted(range(len(deta_f)), key=lambda k: deta_f[k], reverse=True)
        d = random.randint(self.worst_d_min, self.worst_d_max)
        remove_list = sorted_id[:d]
        return remove_list

    def createRandomRepair(self, remove_list, model, sol):
        unassigned_nodes_seq = []
        assigned_nodes_seq = []
        for i in range(len(model.node_list)):
            if i in remove_list:
                unassigned_nodes_seq.append(sol.nodes_seq[i])
            else:
                assigned_nodes_seq.append(sol.nodes_seq[i])

        for node_no in unassigned_nodes_seq:
            index = random.randint(0, len(assigned_nodes_seq) - 1)
            assigned_nodes_seq.insert(index, node_no)
        new_sol = Sol()
        new_sol.nodes_seq = copy.deepcopy(assigned_nodes_seq)
        new_sol.vehicle_num, new_sol.routes, new_sol.distance = self.calObj(assigned_nodes_seq, model)
        return new_sol

    def createGreedyRepair(self, remove_list, model, sol):
        unassigned_nodes_seq = []
        assigned_nodes_seq = []
        for i in range(len(model.node_list)):
            if i in remove_list:
                unassigned_nodes_seq.append(sol.nodes_seq[i])
            else:
                assigned_nodes_seq.append(sol.nodes_seq[i])

        while len(unassigned_nodes_seq) > 0:
            insert_node_no, insert_index = self.findGreedyInsert(unassigned_nodes_seq, assigned_nodes_seq, model)
            assigned_nodes_seq.insert(insert_index, insert_node_no)
            unassigned_nodes_seq.remove(insert_node_no)
        new_sol = Sol()
        new_sol.nodes_seq = copy.deepcopy(assigned_nodes_seq)
        new_sol.vehicle_num, new_sol.routes, new_sol.distance = self.calObj(assigned_nodes_seq, model)
        return new_sol

    def findGreedyInsert(self, unassigned_nodes_seq, assigned_nodes_seq, model):
        best_insert_node_no = None
        best_insert_index = None
        best_insert_cost = float('inf')
        assigned_nodes_seq_vehicle_num, _, _ = self.calObj(assigned_nodes_seq, model)
        for node_no in unassigned_nodes_seq:
            for i in range(len(assigned_nodes_seq)):
                assigned_nodes_seq_ = copy.deepcopy(assigned_nodes_seq)
                assigned_nodes_seq_.insert(i, node_no)
                vehicle_num_, _, _ = self.calObj(assigned_nodes_seq_, model)
                deta_f = vehicle_num_ - assigned_nodes_seq_vehicle_num
                if deta_f < best_insert_cost:
                    best_insert_index = i
                    best_insert_node_no = node_no
                    best_insert_cost = deta_f
        return best_insert_node_no, best_insert_index

    def createRegretRepair(self, remove_list, model, sol):
        unassigned_nodes_seq = []
        assigned_nodes_seq = []
        for i in range(len(model.node_list)):
            if i in remove_list:
                unassigned_nodes_seq.append(sol.nodes_seq[i])
            else:
                assigned_nodes_seq.append(sol.nodes_seq[i])

        while len(unassigned_nodes_seq) > 0:
            insert_node_no, insert_index = self.findRegretInsert(unassigned_nodes_seq, assigned_nodes_seq, model)
            assigned_nodes_seq.insert(insert_index, insert_node_no)
            unassigned_nodes_seq.remove(insert_node_no)
        new_sol = Sol()
        new_sol.nodes_seq = copy.deepcopy(assigned_nodes_seq)
        new_sol.vehicle_num, new_sol.routes, new_sol.distance = self.calObj(assigned_nodes_seq, model)
        return new_sol

    def findRegretInsert(self, unassigned_nodes_seq, assigned_nodes_seq, model):
        opt_insert_node_no = None
        opt_insert_index = None
        opt_insert_cost = -float('inf')
        for node_no in unassigned_nodes_seq:
            n_insert_cost = np.zeros((len(assigned_nodes_seq), 3))
            for i in range(len(assigned_nodes_seq)):
                assigned_nodes_seq_ = copy.deepcopy(assigned_nodes_seq)
                assigned_nodes_seq_.insert(i, node_no)
                vehicle_num_, _, _ = self.calObj(assigned_nodes_seq_, model)
                n_insert_cost[i, 0] = node_no
                n_insert_cost[i, 1] = i
                n_insert_cost[i, 2] = vehicle_num_
            n_insert_cost = n_insert_cost[n_insert_cost[:, 2].argsort()]  # argsort将x中的元素从小到大排列，提取其对应的index(索引)，然后输出到y
            deta_f = 0
            for i in range(1, self.regret_n):
                deta_f = deta_f + n_insert_cost[i, 2] - n_insert_cost[0, 2]
            if deta_f > opt_insert_cost:
                opt_insert_node_no = int(n_insert_cost[0, 0])
                opt_insert_index = int(n_insert_cost[0, 1])
                opt_insert_cost = deta_f
        return opt_insert_node_no, opt_insert_index

    # 更新算子权重
    """
    对于算子权重的更新有两种策略，一种是每执行依次destory和repair更新一次，
    另一种是每执行pu次destory和repair更新一次权重。前者，能够保证权重及时得到更新，
    但却需要更多的计算时间；后者，通过合理设置pu参数，节省了计算时间，同时又不至于权
    重更新太滞后。这里采用后者更新策略。
    """
    def updateWeight(self):

        for i in range(self.d_weight.shape[0]):
            if self.d_select[i] > 0:
                self.d_weight[i] = self.d_weight[i] * (1 - self.rho) + self.rho * self.d_score[i] / self.d_select[
                    i]
            else:
                self.d_weight[i] = self.d_weight[i] * (1 - self.rho)
        for i in range(self.r_weight.shape[0]):
            if self.r_select[i] > 0:
                self.r_weight[i] = self.r_weight[i] * (1 - self.rho) + self.rho * self.r_score[i] / self.r_select[
                    i]
            else:
                self.r_weight[i] = self.r_weight[i] * (1 - self.rho)
        self.d_history_select = self.d_history_select + self.d_select
        self.d_history_score = self.d_history_score + self.d_score
        self.r_history_select = self.r_history_select + self.r_select
        self.r_history_score = self.r_history_score + self.r_score
        
    def run(self, model, epochs=100, freq=5, phi=0.9):
        self.initParam(model)
        history_best_vehicle_num = []
        current_sol_list = []
        history_best_distance = []
        sol = Sol()
        sol.nodes_seq = self.genInitialSol(model)
        sol.vehicle_num, sol.routes, sol.distance = self.calObj(sol.nodes_seq, model)
        model.best_sol = copy.deepcopy(sol)
        history_best_vehicle_num.append(sol.vehicle_num)
        history_best_distance.append(sol.distance)
        for ep in range(epochs):
            T = sol.vehicle_num * 0.2
            self.resetScore()  # 重置算子得分
            for k in range(freq):
                destory_id, repair_id = self.selectDestoryRepair()  # 选择算子
                self.d_select[destory_id] += 1
                self.r_select[repair_id] += 1
                reomve_list = self.doDestory(destory_id, model, sol)  # 执行destory算子
                new_sol = self.doRepair(repair_id, reomve_list, model, sol)  # 执行repair算子
                if new_sol.vehicle_num < sol.vehicle_num:
                    sol = copy.deepcopy(new_sol)
                    if new_sol.vehicle_num < model.best_sol.vehicle_num:
                        model.best_sol = copy.deepcopy(new_sol)
                        self.d_score[destory_id] += self.r1
                        self.r_score[repair_id] += self.r1
                    else:
                        self.d_score[destory_id] += self.r2
                        self.r_score[repair_id] += self.r2

                elif new_sol.vehicle_num == sol.vehicle_num and new_sol.distance < sol.distance:
                    sol = copy.deepcopy(new_sol)
                    if new_sol.vehicle_num == model.best_sol.vehicle_num and new_sol.distance < model.best_sol.distance:
                        model.best_sol = copy.deepcopy(new_sol)
                        self.d_score[destory_id] += self.r1
                        self.r_score[repair_id] += self.r1
                    else:
                        self.d_score[destory_id] += self.r2
                        self.r_score[repair_id] += self.r2

                elif new_sol.vehicle_num - sol.vehicle_num < T:
                    sol = copy.deepcopy(new_sol)
                    self.d_score[destory_id] += self.r3
                    self.r_score[repair_id] += self.r3
                current_sol_list.append(sol.vehicle_num)
                T = T * phi
                print(f"{ep}/{epochs}:{k}/{freq}, best obj:{model.best_sol.vehicle_num}, distance:{model.best_sol.distance}")
                history_best_vehicle_num.append(model.best_sol.vehicle_num)
                history_best_distance.append(model.best_sol.distance)
            self.updateWeight()  # 更新算子权重

        plotObj(history_best_distance)
        plotRoutes(model, name='ALNS')
        plotTSP(model, name='ALNS')


class ANT:
    def __init__(self):
        self.popsize = 300
        self.alpha = 0.5  # 信息素启发因子
        self.beta = 3  # 贪婪启发因子
        self.Q = 0.05
        self.rho = 0.6
        self.tau = {}  # 网络弧信息素
        self.initial_pheromone = 10

    def initial(self, model, pheromone):
        for i in range(len(model.node_list)):
            for j in range(i + 1, len(model.node_list)):
                d = math.sqrt((model.node_list[i].x_coord - model.node_list[j].x_coord) ** 2
                              + (model.node_list[i].y_coord - model.node_list[j].y_coord) ** 2)
                model.distance[i, j] = d
                model.distance[j, i] = d
                self.tau[i, j] = pheromone
                self.tau[j, i] = pheromone

    def move_position(self, model):
        sol_list = []
        local_sol = Sol()
        local_sol.vehicle_num = float('inf')
        local_sol.distance = float('inf')
        for k in range(self.popsize):
            nodes_seq = [int(random.randint(0, len(model.node_list) - 1))]
            all_nodes_seq = copy.deepcopy(model.node_id_list)
            all_nodes_seq.remove(nodes_seq[-1])
            while len(all_nodes_seq) > 0:
                next_node_no = self.search_next_node(model, nodes_seq[-1], all_nodes_seq)
                nodes_seq.append(next_node_no)
                all_nodes_seq.remove(next_node_no)
            sol = Sol()
            sol.nodes_seq = nodes_seq
            sol.vehicle_num, sol.routes, sol.distance = self.cal_obj(sol.nodes_seq, model)
            sol_list.append(sol)
            if sol.vehicle_num < local_sol.vehicle_num:
                local_sol = copy.deepcopy(sol)
            elif sol.vehicle_num == local_sol.vehicle_num and sol.distance < local_sol.distance:
                local_sol = copy.deepcopy(sol)
        model.sol_list = copy.deepcopy(sol_list)
        if local_sol.vehicle_num < model.best_sol.vehicle_num:
            model.best_sol = copy.deepcopy(local_sol)
        elif local_sol.vehicle_num == model.best_sol.vehicle_num and local_sol.distance < model.best_sol.distance:
            model.best_sol = copy.deepcopy(local_sol)

    def move_position_(self, model):
        sol_list = []
        local_sol = Sol()
        local_sol.vehicle_num = float('inf')
        local_sol.distance = float('inf')
        for k in range(self.popsize):
            nodes_seq = [int(random.randint(0, len(model.node_list) - 1))]
            all_nodes_seq = copy.deepcopy(model.node_id_list)
            all_nodes_seq.remove(nodes_seq[-1])
            while len(all_nodes_seq) > 0:
                next_node_no = self.search_next_node(model, nodes_seq[-1], all_nodes_seq)
                nodes_seq.append(next_node_no)
                all_nodes_seq.remove(next_node_no)
            sol = Sol()
            sol.nodes_seq = nodes_seq   # 原始nodes_seq
            sol.num_vehicle_list, sol.vehicle_routes_list, sol.nodes_seq_list, sol.distance_list = \
                self.cal_obj_(sol.nodes_seq, model)
            index = sol.distance_list.index(min(sol.distance_list))
            sol.nodes_seq = sol.nodes_seq_list[index]   # 最优nodes_seq
            sol.vehicle_num = sol.num_vehicle_list[index]
            sol.routes = sol.vehicle_routes_list[index]

            sol.distance = sol.distance_list[index]
            sol_list.append(sol)

            if sol.vehicle_num < local_sol.vehicle_num:
                local_sol = copy.deepcopy(sol)
            elif sol.vehicle_num == local_sol.vehicle_num and sol.distance < local_sol.distance:
                local_sol = copy.deepcopy(sol)
        model.sol_list = copy.deepcopy(sol_list)
        if local_sol.vehicle_num < model.best_sol.vehicle_num:
            model.best_sol = copy.deepcopy(local_sol)
        elif local_sol.vehicle_num == model.best_sol.vehicle_num and local_sol.distance < model.best_sol.distance:
            model.best_sol = copy.deepcopy(local_sol)

    def search_next_node(self, model, current_node_no, all_nodes_seq):
        prob = np.zeros(len(all_nodes_seq))
        for i, node_no in enumerate(all_nodes_seq):
            eta = 1 / model.distance[current_node_no, node_no]
            tau = self.tau[current_node_no, node_no]
            prob[i] = ((tau ** self.alpha) * (eta ** self.beta))

        cumsumprob = (prob / sum(prob)).cumsum()
        cumsumprob -= random.random()
        next_node_no = all_nodes_seq[list(cumsumprob > 0).index(True)]
        return next_node_no

    def cal_obj(self, nodes_seq, model):
        num_vehicle, vehicle_routes = self.splitRoutes(nodes_seq, model)
        distance = 0
        for route in vehicle_routes:
            distance += self.calDistance(route, model)
        return num_vehicle, vehicle_routes, distance

    def cal_obj_(self, nodes_seq, model):
        distance_list = []
        num_vehicle_list, vehicle_routes_list, nodes_seq_list = self.splitRoutes_(nodes_seq, model)
        for vehicle_routes in vehicle_routes_list:
            distance = 0
            for route in vehicle_routes:
                # 4.7 修改 error:index out of list ｜ 90需求点vehicle_routes出现空列表
                if len(route) == 0:
                    vehicle_routes.remove(route)
                    continue

                distance += self.calDistance(route, model)

            distance_list.append(distance)
        return num_vehicle_list, vehicle_routes_list, nodes_seq_list, distance_list

    @staticmethod
    def splitRoutes(nodes_seq, model):
        num_vehicle = 0
        vehicle_routes = []
        route = []
        remained_cap = model.vehicle_cap
        for node_no in nodes_seq:
            if remained_cap - model.node_list[node_no].max_demand >= 0:
                route.append(node_no)
                remained_cap = remained_cap - model.node_list[node_no].max_demand
            else:
                vehicle_routes.append(route)
                route = [node_no]
                num_vehicle = num_vehicle + 1
                remained_cap = model.vehicle_cap - model.node_list[node_no].max_demand
        vehicle_routes.append(route)
        num_vehicle = num_vehicle + 1
        return num_vehicle, vehicle_routes

    @staticmethod
    def splitRoutes_(nodes_seq, model):
        nodes_seq_list, vehicle_routes_list, num_vehicle_list = [], [], []
        for i in range(len(nodes_seq)):
            num_vehicle = 0
            vehicle_routes = []
            route = []
            remained_cap = model.vehicle_cap
            for node_no in nodes_seq:
                if remained_cap - model.node_list[node_no].max_demand >= 0:
                    route.append(node_no)
                    remained_cap = remained_cap - model.node_list[node_no].max_demand
                else:
                    vehicle_routes.append(route)
                    route = [node_no]
                    num_vehicle = num_vehicle + 1
                    remained_cap = model.vehicle_cap - model.node_list[node_no].max_demand
            vehicle_routes.append(route)
            num_vehicle = num_vehicle + 1
            nodes_seq_list.append(copy.deepcopy(nodes_seq))
            vehicle_routes_list.append(vehicle_routes)
            num_vehicle_list.append(num_vehicle)
            nodes_seq.append(nodes_seq.pop(0))

        return num_vehicle_list, vehicle_routes_list, nodes_seq_list

    @staticmethod
    def calDistance(route, model):
        distance = 0
        school = model.school
        for i in range(len(route) - 1):
            from_node = model.node_list[route[i]]
            to_node = model.node_list[route[i + 1]]
            distance += math.sqrt(
                (from_node.x_coord - to_node.x_coord) ** 2 + (from_node.y_coord - to_node.y_coord) ** 2)
        first_node = model.node_list[route[0]]
        last_node = model.node_list[route[-1]]
        distance += math.sqrt((first_node.x_coord - school.x_coord) ** 2 + (first_node.y_coord - school.y_coord) ** 2)
        distance += math.sqrt((last_node.x_coord - school.x_coord) ** 2 + (last_node.y_coord - school.y_coord) ** 2)
        return distance

    def update_tau(self, model):
        # for sol in model.sol_list:
        #     print(sol.distance)
        rho = self.rho
        for k in self.tau.keys():
            self.tau[k] *= (1 - rho)
        for sol in model.sol_list:

            nodes_seq = sol.nodes_seq
            for i in range(len(nodes_seq) - 1):
                from_node_no = nodes_seq[i]
                to_node_no = nodes_seq[i + 1]
                self.tau[from_node_no, to_node_no] += self.Q / sol.distance

    def run(self, model, no=None, initial_pheromone=10, epochs=100):
        sol = Sol()
        sol.distance = float('inf')
        sol.vehicle_num = float('inf')
        model.best_sol = sol
        history_best_obj = []
        self.initial(model, initial_pheromone)
        for ep in range(epochs):
            self.move_position_(model)
            self.update_tau(model)
            history_best_obj.append(model.best_sol.distance)
            print(f'{ep}/{epochs}   vehicle:{model.best_sol.vehicle_num}    distance:{model.best_sol.distance}')
        print(model.best_sol.nodes_seq)
        print(model.best_sol.routes)
        print(model.best_sol.distance_list)
        print(model.best_sol.num_vehicle_list)
        plotRoutes(model, no=no, name='ant')
        plotTSP(model, no=no, name='ant')


class GENE:
    def __init__(self):
        self.pc = 0.5  # 交叉概率
        self.pm = 0.2  # 变异概率
        self.popsize = 100
        self.n_select = 80  # 优良个体选择数量

    def genInitialSol(self, model):
        nodes_seq = copy.deepcopy(model.node_id_list)
        for i in range(self.popsize):
            seed = random.randint(0, 101)
            random.seed(seed)
            random.shuffle(nodes_seq)
            sol = Sol()
            sol.nodes_seq = copy.deepcopy(nodes_seq)
            model.sol_list.append(sol)

    def calFit(self, model):
        """适应度=种群中每个个体与表现最差的个体的解的差值"""
        worst_obj = -float('inf')
        best_sol = Sol()  # 记录局部最优解
        best_sol.distance = float('inf')
        best_sol.vehicle_num = float('inf')
        # 计算目标函数
        for sol in model.sol_list:
            nodes_seq = sol.nodes_seq
            num_vehicle, vehicle_routes = self.splitRoutes(nodes_seq, model)

            distance = 0
            for route in vehicle_routes:
                distance += self.calDistance(route, model)
            sol.distance = distance
            sol.routes = vehicle_routes
            sol.vehicle_num = num_vehicle
            if sol.distance > worst_obj:
                worst_obj = sol.distance
            if sol.distance < best_sol.distance:
                best_sol = copy.deepcopy(sol)
        # 计算适应度
        for sol in model.sol_list:
            sol.fit = worst_obj - sol.distance
        # 更新全局最优解
        if best_sol.distance< model.best_sol.distance:
            model.best_sol = best_sol

    def calFit_(self, model):
        """适应度=种群中每个个体与表现最差的个体的解的差值"""
        worst_obj = -float('inf')
        best_sol = Sol()  # 记录局部最优解
        best_sol.distance = float('inf')
        best_sol.vehicle_num = float('inf')
        # 计算目标函数
        for sol in model.sol_list:
            distance_list = []
            nodes_seq = sol.nodes_seq
            num_vehicle_list, vehicle_routes_list, nodes_seq_list = self.splitRoutes_(nodes_seq, model)

            for vehicle_routes in vehicle_routes_list:
                distance = 0
                for route in vehicle_routes:
                    if len(route) == 0:
                        vehicle_routes.remove(route)
                        continue
                    distance += self.calDistance(route, model)
                distance_list.append(distance)
            index = distance_list.index(min(distance_list))
            sol.nodes_seq = nodes_seq_list[index]  # 最优nodes_seq
            sol.vehicle_num = num_vehicle_list[index]
            sol.routes = vehicle_routes_list[index]
            sol.distance = distance_list[index]

            if sol.distance > worst_obj:
                worst_obj = sol.distance
            if sol.distance < best_sol.distance:
                best_sol = copy.deepcopy(sol)
        # 计算适应度
        for sol in model.sol_list:
            sol.fit = worst_obj - sol.distance
        # 更新全局最优解
        if best_sol.distance< model.best_sol.distance:
            model.best_sol = best_sol

    @staticmethod
    def splitRoutes(nodes_seq, model):
        num_vehicle = 0
        vehicle_routes = []
        route = []
        remained_cap = model.vehicle_cap
        for node_no in nodes_seq:
            if remained_cap - model.node_list[node_no].max_demand >= 0:
                route.append(node_no)
                remained_cap = remained_cap - model.node_list[node_no].max_demand
            else:
                vehicle_routes.append(route)
                route = [node_no]
                num_vehicle = num_vehicle + 1
                remained_cap = model.vehicle_cap - model.node_list[node_no].max_demand
        vehicle_routes.append(route)
        num_vehicle += 1
        return num_vehicle, vehicle_routes

    @staticmethod
    def splitRoutes_(nodes_seq, model):
        nodes_seq_list, vehicle_routes_list, num_vehicle_list = [], [], []
        for i in range(len(nodes_seq)):
            num_vehicle = 0
            vehicle_routes = []
            route = []
            remained_cap = model.vehicle_cap
            for node_no in nodes_seq:
                if remained_cap - model.node_list[node_no].max_demand >= 0:
                    route.append(node_no)
                    remained_cap = remained_cap - model.node_list[node_no].max_demand
                else:
                    vehicle_routes.append(route)
                    route = [node_no]
                    num_vehicle = num_vehicle + 1
                    remained_cap = model.vehicle_cap - model.node_list[node_no].max_demand
            vehicle_routes.append(route)
            num_vehicle = num_vehicle + 1
            nodes_seq_list.append(copy.deepcopy(nodes_seq))
            vehicle_routes_list.append(vehicle_routes)
            num_vehicle_list.append(num_vehicle)
            nodes_seq.append(nodes_seq.pop(0))

        return num_vehicle_list, vehicle_routes_list, nodes_seq_list

    @staticmethod
    def calDistance(route, model):
        distance = 0
        school = model.school
        for i in range(len(route) - 1):
            from_node = model.node_list[route[i]]
            to_node = model.node_list[route[i + 1]]
            distance += math.sqrt(
                (from_node.x_coord - to_node.x_coord) ** 2 + (from_node.y_coord - to_node.y_coord) ** 2)
        first_node = model.node_list[route[0]]
        last_node = model.node_list[route[-1]]
        distance += math.sqrt((first_node.x_coord - school.x_coord) ** 2 + (first_node.y_coord - school.y_coord) ** 2)
        distance += math.sqrt((last_node.x_coord - school.x_coord) ** 2 + (last_node.y_coord - school.y_coord) ** 2)
        return distance

    def selectSol(self, model):
        """每次随机选出两个个体(解)，选择其中较好的那一个。共选出n_select个优秀个体"""
        sol_list = copy.deepcopy(model.sol_list)
        model.sol_list = []
        for i in range(self.n_select):
            f1_index = random.randint(0, len(sol_list) - 1)
            f2_index = random.randint(0, len(sol_list) - 1)
            f1_fit = sol_list[f1_index].fit
            f2_fit = sol_list[f2_index].fit
            if f1_fit < f2_fit:
                model.sol_list.append(sol_list[f2_index])
            else:
                model.sol_list.append(sol_list[f1_index])

    def crossSol(self, model):
        """
        交叉规则:选取两个不同的亲代，判断是否发生交叉，如否，添加这两个亲代到下一代种群，如是，
        选取这两个亲代中相同索引的一段元素，两个子代分别继承两个亲代的这两段元素(元素和索引都完全相同)
        剩下的元素由另一个亲代去除已继承的元素后按顺序补位构成。循环直到达到下一代种群规模。
        """
        sol_list = copy.deepcopy(model.sol_list)
        model.sol_list = []
        while True:
            f1_index = random.randint(0, len(sol_list) - 1)
            f2_index = random.randint(0, len(sol_list) - 1)
            if f1_index != f2_index:
                f1 = copy.deepcopy(sol_list[f1_index])
                f2 = copy.deepcopy(sol_list[f2_index])
                if random.random() <= self.pc:
                    cro1_index = int(random.randint(0, len(model.node_list) - 1))
                    cro2_index = int(random.randint(cro1_index, len(model.node_list) - 1))
                    new_c1_f = []
                    new_c1_m = f1.nodes_seq[cro1_index:cro2_index + 1]
                    new_c1_b = []
                    new_c2_f = []
                    new_c2_m = f2.nodes_seq[cro1_index:cro2_index + 1]
                    new_c2_b = []
                    for index in range(len(model.node_list)):
                        if len(new_c1_f) < cro1_index:
                            if f2.nodes_seq[index] not in new_c1_m:
                                new_c1_f.append(f2.nodes_seq[index])
                        else:
                            if f2.nodes_seq[index] not in new_c1_m:
                                new_c1_b.append(f2.nodes_seq[index])
                    for index in range(len(model.node_list)):
                        if len(new_c2_f) < cro1_index:
                            if f1.nodes_seq[index] not in new_c2_m:
                                new_c2_f.append(f1.nodes_seq[index])
                        else:
                            if f1.nodes_seq[index] not in new_c2_m:
                                new_c2_b.append(f1.nodes_seq[index])
                    new_c1 = copy.deepcopy(new_c1_f)
                    new_c1.extend(new_c1_m)
                    new_c1.extend(new_c1_b)
                    f1.nodes_seq = new_c1
                    new_c2 = copy.deepcopy(new_c2_f)
                    new_c2.extend(new_c2_m)
                    new_c2.extend(new_c2_b)
                    f2.nodes_seq = new_c2
                    model.sol_list.append(copy.deepcopy(f1))
                    model.sol_list.append(copy.deepcopy(f2))
                else:
                    model.sol_list.append(copy.deepcopy(f1))
                    model.sol_list.append(copy.deepcopy(f2))
                if len(model.sol_list) > self.popsize:
                    break

    def muSol(self, model):
        sol_list = copy.deepcopy(model.sol_list)
        model.sol_list = []
        while True:
            f1_index = int(random.randint(0, len(sol_list) - 1))
            f1 = copy.deepcopy(sol_list[f1_index])
            m1_index = random.randint(0, len(model.node_list) - 1)
            m2_index = random.randint(0, len(model.node_list) - 1)
            if m1_index != m2_index:
                if random.random() <= self.pm:
                    f1.nodes_seq[m1_index], f1.nodes_seq[m2_index] = f1.nodes_seq[m2_index], f1.nodes_seq[m1_index]
                    model.sol_list.append(copy.deepcopy(f1))
                else:
                    model.sol_list.append(copy.deepcopy(f1))
                if len(model.sol_list) > self.popsize:
                    break

    def run(self, model, no=None, epochs=1000):
        self.genInitialSol(model)
        best_sol = Sol()
        best_sol.vehicle_num = float('inf')
        best_sol.distance = float('inf')
        model.best_sol = best_sol
        for ep in range(epochs):
            self.calFit_(model)  # 计算适应度
            self.selectSol(model)  # 优良个体选择
            self.crossSol(model)  # 交叉
            self.muSol(model)  # 突变
            print(f"{ep}/{epochs} vehicle:{model.best_sol.vehicle_num}  distance:{model.best_sol.distance}")
        print(model.best_sol.nodes_seq)
        print(model.best_sol.routes)
        plotRoutes(model, name='gene', no=no)
        plotTSP(model, name='gene', no=no)

