"""
作者：zhaozhijie
日期：2022年10月26日17时52分
"""
import copy
import math
import random
from tsp import Model
from functools import wraps
from tools import plotOptimRoutes
from utils import ANT
import matplotlib.pyplot as plt


class Line:
    def __init__(self, r1, r2):
        self.primary = r1
        self.additional = r2
        self.extended = self.primary + self.additional
        self.real_route = None


class Vehicle(Line):
    def __init__(self, *args, **kwargs):
        super(Vehicle, self).__init__(*args, **kwargs)
        self.s = None   # 车辆服务的第一个节点
        self.w = 0      # 车辆主要路线的负载量
        self.e = 0      # 车辆服务完主要路线后的剩余容量
        self.e_ = 0     # 车辆实际用于服务额外路线的容量
        self.c = 0      # 车辆实际负载
        # 2.21更新
        self.d = 0      # 车辆实际行驶距离
        self.t = 0      # 车辆实际行驶时间


class VRPSD:
    def __init__(self, model, k=1, threshold=10000):
        self.k = k
        self.threshold = threshold
        self.model = model
        self.vehicles = []
        self.extend_lines = []
        self.optim_routes = []
        self.assign_line()
        self.propagation()
        self.optim_routes = self.extract_route()

    def assign_line(self):
        # 根据VRP为每辆校车分配扩展路线
        for index, route in enumerate(self.model.best_sol.routes):
            addition_route = []
            try:
                for i in range(self.k):
                    addition_route += self.model.best_sol.routes[index+i+1]
                vehicle = Vehicle(route, addition_route)
            except IndexError:
                vehicle = Vehicle(route, addition_route)
            self.vehicles.append(copy.deepcopy(vehicle))
            self.extend_lines.append(vehicle.extended)

    def propagation(self):
        # 第一个
        index = self.vehicles[0].primary[0]
        self.vehicles[0].s = self.model.node_list[index]    # 第一辆车的起始节点

        for no in self.vehicles[0].primary:                 # 主要路线负载量
            self.vehicles[0].w += self.model.node_list[no].real_demand

        self.vehicles[0].e = self.model.vehicle_cap - self.vehicles[0].w    # 服务完主要路线后的剩余容量

        cusum_w = 0         # 额外路线的实际需求
        for i in range(self.k):
            try:
                for no in self.vehicles[i+1].primary:
                    self.vehicles[i+1].w += self.model.node_list[no].real_demand
            except IndexError:
                exit("k值过大")
            cusum_w += self.vehicles[i+1].w
        self.vehicles[0].e_ = min(self.vehicles[0].e, cusum_w)      # 用于服务额外路线的容量

        fulfilled = 0
        count = 0
        if self.model.node_list[self.vehicles[1].primary[0]].real_demand > self.vehicles[0].e_:
            self.vehicles[1].s = self.model.node_list[self.vehicles[1].primary[0]]
            self.vehicles[0].real_route = self.vehicles[0].primary
        else:
            current_route = copy.deepcopy(self.vehicles[0].primary)
            for j in range(self.k):
                i = 0
                if fulfilled >= self.vehicles[0].e_:
                    break
                start = self.vehicles[j+1].primary[i]   # 额外路线的开始节点
                fulfilled += self.model.node_list[start].real_demand
                # 2.21新增最大乘坐时间约束
                current_route.append(start)
                dis = ANT.calDistance(current_route, model)
                with open(f_name, mode="a") as f:
                    print(f"当前总行驶距离为{dis}")
                    f.write(f"当前总行驶距离为{dis}\n")

                while fulfilled <= self.vehicles[0].e_ and dis < self.threshold:

                    try:
                        i += 1
                        start = self.vehicles[j+1].primary[i]
                        current_route.append(start)
                        fulfilled += self.model.node_list[start].real_demand
                        dis = ANT.calDistance(current_route, model)
                        with open(f_name, mode="a") as f:
                            print(f"当前总行驶距离为{dis}")
                            f.write(f"当前总行驶距离为{dis}\n")
                    except IndexError:
                        count += 1
                        break

            if count == self.k:
                self.vehicles[1].s = self.model.node_list[self.vehicles[1+self.k].primary[0]]
                self.vehicles[0].real_route = self.vehicles[0].primary + self.vehicles[0].additional
            elif count == 0:
                self.vehicles[1].s = self.model.node_list[self.vehicles[1].primary[i+1]]
                self.vehicles[0].real_route = self.vehicles[0].primary + self.vehicles[1].primary[:i+1]
            else:
                self.vehicles[1].s = self.model.node_list[self.vehicles[count+1].primary[i]]
                r = []
                for n in range(count+1):
                    r += self.vehicles[0+n].primary
                self.vehicles[0].real_route = r + self.vehicles[count+1].primary[:i]
        for index in self.vehicles[0].real_route:
            self.vehicles[0].c += self.model.node_list[index].real_demand

        # 循环
        for i, vehicle in enumerate(self.vehicles[1:]):
            with open(f_name, mode="a") as f:
                print("-"*40)
                f.write("-"*40 + "\n")

            if vehicle.s is None:
                break
            index = i+1     # 当前vehicle在列表中的位置
            print(vehicle.extended, vehicle.s.id)
            try:
                ini = vehicle.extended.index(vehicle.s.id)
            except ValueError:
                exit("riding time failure")
            current_route = []
            dis = 0
            for j, node_no in enumerate(vehicle.extended[ini:]):
                if vehicle.c > model.vehicle_cap or dis >= self.threshold:
                    vehicle.c -= model.node_list[vehicle.extended[ini:][j-1]].real_demand
                    next_s_index = model.best_sol.nodes_seq.index(vehicle.extended[ini:][j-1])
                    self.vehicles[index+1].s = model.node_list[model.best_sol.nodes_seq[next_s_index]]

                    end = vehicle.extended.index(vehicle.extended[ini:][j-1])
                    vehicle.real_route = vehicle.extended[ini:end]
                    break
                else:
                    vehicle.c += model.node_list[node_no].real_demand
                    # 2.21新增最大乘坐时间约束
                    current_route.append(model.node_list[node_no].id)
                    dis = ANT.calDistance(current_route, model)
                    with open(f_name, mode="a") as f:
                        print(f"当前总行驶距离为{dis}")
                        f.write(f"当前总行驶距离为{dis}\n")

            if vehicle.c > model.vehicle_cap:
                vehicle.c -= model.node_list[vehicle.extended[ini:][-1]].real_demand
                next_s_index = model.best_sol.nodes_seq.index(vehicle.extended[ini:][-1])+1
                try:
                    self.vehicles[index + 1].s = model.node_list[model.best_sol.nodes_seq[next_s_index]]

                except IndexError:
                    self.vehicles[index + 1].s = None
                vehicle.real_route = vehicle.extended[ini:]
            elif vehicle.c <= model.vehicle_cap and vehicle.real_route is None:

                next_s_index = model.best_sol.nodes_seq.index(vehicle.extended[ini:][-1]) + 1
                try:
                    self.vehicles[index + 1].s = model.node_list[model.best_sol.nodes_seq[next_s_index]]
                except IndexError:
                    try:
                        self.vehicles[index + 1].s = None
                    except IndexError:
                        pass
                vehicle.real_route = vehicle.extended[ini:]

        try:
            try:
                if self.vehicles[index+1].s.id is not None:
                    ini = self.vehicles[index+1].extended.index(self.vehicles[index+1].s.id)
                    self.vehicles[index+1].real_route = self.vehicles[index+1].extended[ini:]
                    for node_no in self.vehicles[index+1].real_route:
                        self.vehicles[index+1].c += model.node_list[node_no].real_demand
            except IndexError:
                pass
        except AttributeError:
            pass

    def extract_route(self):
        routes = []
        print(model.best_sol.nodes_seq)
        for vehicle in self.vehicles:
            dis = 0
            try:
                dis = ANT.calDistance(vehicle.real_route, model)
            except TypeError:
                pass
            with open(f_name, mode="a") as f:
                print(f"扩展路线:{vehicle.extended}  实际路线:{vehicle.real_route} 实际荷载:{vehicle.c} 实际行驶距离:{dis}", )
                f.write(f"扩展路线:{vehicle.extended}  实际路线:{vehicle.real_route} 实际荷载:{vehicle.c} 实际行驶距离:{dis}\n")

            if vehicle.real_route is not None:
                routes.append(vehicle.real_route)
        return routes


def tips(func):
    @wraps(func)
    def wrapper(*args, **kwargs):
        func(*args, **kwargs)
        with open(f_name, mode="a") as f:
            print(f"节点id:{node.id} 节点最大需求:{node.max_demand} 节点真实需求:{node.real_demand}")
            f.write(f"节点id:{node.id} 节点最大需求:{node.max_demand} 节点真实需求:{node.real_demand}\n")

    return wrapper


@tips
def simulate(node):
    mu = node.max_demand // 2
    sigma = round(math.sqrt(mu))
    temp = round(random.gauss(mu, sigma))
    temp = max(0, temp)
    temp = min(node.max_demand, temp)
    node.real_demand = temp

# 均匀分布
@tips
def Uniform_Distribution(node):
    b = node.max_demand
    a = 0
    node.real_demand = random.randint(a, b)

# 高需求正态分布
@tips
def Gauss_Major(node):
    mu = round(node.max_demand*0.9)
    sigma = round(node.max_demand-mu/3)
    temp = round(random.gauss(mu, sigma))
    temp = max(0, temp)
    temp = min(node.max_demand, temp)
    node.real_demand = temp

# 低需求正态分布
@tips
def Gauss_Minor(node):
    mu = round(node.max_demand*0.25)
    sigma = round(mu/3)
    temp = round(random.gauss(mu, sigma))
    temp = max(0, temp)
    temp = min(node.max_demand, temp)
    node.real_demand = temp


if __name__ == '__main__':
    for i in range(4, 5):
        try:
            seed = i
            random.seed(seed)
            optim = 'ANT'
            f_name = optim+f'-log{seed}.txt'
            model = Model.load_obj(f"现实问题/model-{optim}.pkl")
            model.get_sol_info(f_name)
            for node in model.node_list:
                Uniform_Distribution(node)
            solver = VRPSD(model, k=4, threshold=30000)
            plotOptimRoutes(model, solver, 'k', optim, no=10, seed=seed)
            optimDistance = 0
            for route in solver.optim_routes:
                optimDistance += ANT.calDistance(route, model)
            dis_flag = ((model.best_sol.distance-optimDistance)/model.best_sol.distance)*100
            with open(f_name, mode="a") as f:
                print(f"不使用k-overlap策略的总行驶距离为{round(model.best_sol.distance, 1)},使用的车辆数为{model.best_sol.vehicle_num}\n"
                      f"使用{solver.k}-overlap策略的总行驶距离为{round(optimDistance, 1)},使用的车辆数为{len(solver.optim_routes)}\n"
                      f"总距离优化了{round(dis_flag, 2)}%")
                f.write(f"不使用k-overlap策略的总行驶距离为{round(model.best_sol.distance, 1)},使用的车辆数为{model.best_sol.vehicle_num}\n"
                      f"使用{solver.k}-overlap策略的总行驶距离为{round(optimDistance, 1)},使用的车辆数为{len(solver.optim_routes)}\n"
                      f"总距离优化了{round(dis_flag, 2)}%\n")
        except IndexError:
            # exit('不明原因的错误')
            continue
        plt.clf()

