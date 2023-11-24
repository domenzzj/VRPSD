"""
作者：zhaozhijie
日期：2022年10月25日12时18分
"""
import csv
import matplotlib.pyplot as plt
import numpy as np


class Node:
    def __init__(self, x=0, y=0):
        self.id = 0
        self.x_coord = x
        self.y_coord = y
        self.max_demand = 0
        self.real_demand = 0


class Sol:
    def __init__(self):
        self.nodes_seq = None  # 需求节点seq_no有序排列集合，对应TSP的解
        self.vehicle_num = 0  # 优化目标值
        self.routes = None  # 车辆路径集合，对应CVRP的解
        self.distance = 0


def read_csv_file(model, demand_file, school_file):
    with open(demand_file, "r") as f:
        demand_reader = csv.DictReader(f)
        for row in demand_reader:
            node = Node()
            node.id = int(row["id"])
            node.x_coord = float(row["x_coord"])
            node.y_coord = float(row["y_coord"])
            node.max_demand = int(row['demand'])
            model.node_list.append(node)
            model.node_id_list.append(node.id)
            model.demand_dict[node.id] = node

    with open(school_file, "r") as f:
        school_reader = csv.DictReader(f)
        for row in school_reader:
            node = Node()
            node.x_coord = float(row["x_coord"])
            node.y_coord = float(row["y_coord"])
            model.school = node


def plotObj(obj_list):
    plt.rcParams['axes.unicode_minus'] = False
    plt.plot(np.arange(1, len(obj_list) + 1), obj_list)
    plt.xlabel('Iterations')
    plt.ylabel('Obj Value')
    plt.grid()
    plt.xlim(1, len(obj_list) + 1)
    plt.savefig('plot/result1.png')
    plt.show()


def plotRoutes(model, name, no):
    for route in model.best_sol.routes:
        x_coord = [model.school.x_coord]
        y_coord = [model.school.y_coord]
        for node_id in route:
            x_coord.append(model.demand_dict[node_id].x_coord)
            y_coord.append(model.demand_dict[node_id].y_coord)
            plt.text(x_coord[-1], y_coord[-1], str(node_id), verticalalignment='top', horizontalalignment='right',
                     rotation=15)
        x_coord.append(model.school.x_coord)
        y_coord.append(model.school.y_coord)
        plt.plot(x_coord, y_coord, marker='o', color='black', linewidth=0.5, markersize=5)

    plt.plot(model.school.x_coord, model.school.y_coord, marker='o', color='red')
    plt.xlabel('x_coord')
    plt.ylabel('y_coord')
    plt.savefig(f'现实问题/{name}-routes.png')
    plt.show()


def plotTSP(model, name, no):
    x_coord, y_coord = [], []
    for node_id in model.best_sol.nodes_seq:
        x_coord.append(model.demand_dict[node_id].x_coord)
        y_coord.append(model.demand_dict[node_id].y_coord)
        plt.text(x_coord[-1], y_coord[-1], str(node_id), verticalalignment='top', horizontalalignment='right',
                 rotation=15)
    x_coord.append(model.demand_dict[model.best_sol.nodes_seq[0]].x_coord)
    y_coord.append(model.demand_dict[model.best_sol.nodes_seq[0]].y_coord)
    plt.plot(x_coord, y_coord, marker='o', color='black', linewidth=0.5, markersize=5)
    plt.xlabel('x_coord')
    plt.ylabel('y_coord')
    plt.savefig(f'现实问题/{name}-TSP.png')
    plt.show()


def plotOptimRoutes(model, slover, name, optim, no, seed):
    for route in slover.optim_routes:
        x_coord = [model.school.x_coord]
        y_coord = [model.school.y_coord]
        for node_id in route:
            x_coord.append(model.demand_dict[node_id].x_coord)
            y_coord.append(model.demand_dict[node_id].y_coord)
            plt.text(x_coord[-1], y_coord[-1], str(node_id), verticalalignment='top', horizontalalignment='right',
                     rotation=15)
        x_coord.append(model.school.x_coord)
        y_coord.append(model.school.y_coord)

        plt.plot(x_coord, y_coord, marker='o', color='black', linewidth=0.5, markersize=5)

    plt.plot(model.school.x_coord, model.school.y_coord, marker='o', color='red')
    plt.xlabel('x_coord')
    plt.ylabel('y_coord')
    plt.savefig(f'现实问题/{name}-optimRoutes-{optim}{seed}.png')
    # plt.show()
