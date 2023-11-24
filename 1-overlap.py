"""
作者：zhaozhijie
日期：2022年10月26日17时52分
"""
import copy
import random
from tsp import Model
from functools import wraps


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


class VRPSD:
    def __init__(self, model, t=1):
        self.t = t
        self.model = model
        self.vehicles = []
        self.extend_lines = []
        self.assign_line()
        print("扩展路线:", self.extend_lines)
        self.propagation()

    def assign_line(self):
        # 根据VRP为每辆校车分配扩展路线
        for index, route in enumerate(self.model.best_sol.routes):
            addition_route = []
            try:
                for i in range(self.t):
                    addition_route += self.model.best_sol.routes[index+i+1]
                vehicle = Vehicle(route, addition_route)
                new_route = route + addition_route
            except IndexError:
                vehicle = Vehicle(route, addition_route)
                new_route = route + addition_route
            self.vehicles.append(copy.deepcopy(vehicle))
            self.extend_lines.append(new_route)

    def propagation(self):
        # 第一个
        index = self.vehicles[0].primary[0]
        self.vehicles[0].s = self.model.node_list[index]

        for no in self.vehicles[0].primary:
            self.vehicles[0].w += self.model.node_list[no].real_demand

        self.vehicles[0].e = self.model.vehicle_cap - self.vehicles[0].w

        cusum_w = 0
        for i in range(self.t):
            for no in self.vehicles[i+1].primary:
                self.vehicles[i+1].w += self.model.node_list[no].real_demand
            cusum_w += self.vehicles[i+1].w
        self.vehicles[0].e_ = min(self.vehicles[0].e, cusum_w)

        fulfilled = 0
        i = 0
        start = self.vehicles[1].primary[i]
        fulfilled += self.model.node_list[start].real_demand
        if fulfilled > self.vehicles[0].e_:
            self.vehicles[1].s = self.model.node_list[self.vehicles[1].primary[0]]
            self.vehicles[0].real_route = self.vehicles[0].primary
        else:
            while fulfilled <= self.vehicles[0].e_:
                try:
                    i += 1
                    start = self.vehicles[1].primary[i]
                    fulfilled += self.model.node_list[start].real_demand
                except IndexError:
                    self.vehicles[1].s = self.model.node_list[self.vehicles[1].additional[0]]
                    self.vehicles[0].real_route = self.vehicles[0].extended
                    break
                self.vehicles[1].s = self.model.node_list[self.vehicles[1].primary[i]]
                self.vehicles[0].real_route = self.vehicles[0].primary + self.vehicles[0].additional[:i]
        for index in self.vehicles[0].real_route:
            self.vehicles[0].c += self.model.node_list[index].real_demand

        # 遍历
        for i, vehicle in enumerate(self.vehicles[1:-1]):
            index = i+1

            vehicle.e = self.model.vehicle_cap - vehicle.w

            for no in self.vehicles[index+1].primary:
                self.vehicles[index+1].w += self.model.node_list[no].real_demand
            vehicle.e_ = min(vehicle.e, self.vehicles[index+1].w)

            fulfilled = 0
            k = 0
            start = self.vehicles[index+1].primary[k]   # 必须要完成自己的主要路线，所以从下一辆车的第一个主要路线开始计算
            fulfilled += self.model.node_list[start].real_demand    # 计算下一辆车主要路线的累计乘客数量
            if fulfilled > vehicle.e_:                  # 如果下一辆车的第一个节点就超过了本辆车的剩余容量
                self.vehicles[index+1].s = self.model.node_list[self.vehicles[index+1].primary[0]]  # 设定下一辆车的起始节点为下一辆车的主要路线的第一个节点
                try:                                    # 实际路线为主要路线中起始点到最后的切片，如果为空则不出车
                    vehicle.real_route = vehicle.primary[vehicle.primary.index(vehicle.s.id):]
                except ValueError:
                    vehicle.real_route = []
            else:
                while fulfilled <= vehicle.e_:  # 在下一辆车的主要路线上累加，直到超过e_
                    try:
                        k += 1
                        start = self.vehicles[index+1].primary[k]
                        fulfilled += self.model.node_list[start].real_demand
                    except IndexError:      # 如果该辆车能满足自己全部的扩展路线
                        if self.vehicles[index+1].additional:
                            self.vehicles[index+1].s = self.model.node_list[self.vehicles[index+1].additional[0]]   # 下一辆车的起始节点为自身额外路线的第一个节点
                        else:
                            self.vehicles[index + 1].s = None
                        vehicle.real_route = vehicle.extended[vehicle.extended.index(vehicle.s.id):]
                        break
                    self.vehicles[index+1].s = self.model.node_list[self.vehicles[index+1].primary[k]]  # 下一辆车的起始节点为自身主要路线的第k个节点
                    try:
                        primary_slice = vehicle.primary[vehicle.primary.index(vehicle.s.id):]
                    except ValueError:
                        primary_slice = []
                    vehicle.real_route = primary_slice + vehicle.additional[:k]     # 实际服务路线
            for index in vehicle.real_route:
                vehicle.c += self.model.node_list[index].real_demand

        for vehicle in self.vehicles:
            try:
                print(f"{vehicle.real_route} 实际容量:{vehicle.c} 起始点:{vehicle.s.id} 主线路实际需求:{vehicle.w} 空余容量:{vehicle.e} 实际用于额外路线的容量:{vehicle.e_}")
            except AttributeError:
                pass


def tips(func):
    @wraps(func)
    def wrapper(*args, **kwargs):
        res = func(*args, **kwargs)
        print(f"节点id:{node.id} 节点最大需求:{node.max_demand} 节点真实需求:{node.real_demand}")
        return res
    return wrapper


@tips
def simulate(node):
    node.real_demand = random.randint(0, node.max_demand)


if __name__ == '__main__':
    model = Model.load_obj("model.pkl")
    model.get_sol_info()
    for node in model.node_list:
        simulate(node)
    solver = VRPSD(model)
