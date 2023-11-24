"""
作者：zhaozhijie
日期：2022年10月25日09时38分
"""
from utils import ALNS, ANT, GENE
from tools import *
import pickle


class Model:

    def __init__(self):
        self.node_list = []   # *
        self.node_id_list = []
        self.best_sol = None   # *
        self.vehicle_cap = 60
        self.distance = {}
        self.school = Node(2500, 2500)
        self.demand_dict = {}
        self.sol_list = []

    def TSP(self, optim):
        if optim == "ALNS":
            return ALNS()
        elif optim == 'ANT':
            return ANT()
        elif optim == 'GENE':
            return GENE()

    def get_sol_info(self, f_name):
        with open(f_name, mode="a") as f:
            print("TSP:", self.best_sol.nodes_seq)
            print("校车使用数量:", self.best_sol.vehicle_num)
            print("主线路(VRP):", self.best_sol.routes)
            print("总行驶距离:", self.best_sol.distance)
            f.write("TSP:"+str(self.best_sol.nodes_seq)+"\n")
            f.write("校车使用数量:"+str(self.best_sol.vehicle_num) + "\n")
            f.write("主线路(VRP):"+str(self.best_sol.routes) + "\n")
            f.write("总行驶距离:"+str(self.best_sol.distance) + "\n")

    @staticmethod
    def save_obj(obj, name):
        with open(name + '.pkl', 'wb') as f:
            pickle.dump(obj, f, pickle.HIGHEST_PROTOCOL)

    @staticmethod
    def load_obj(file):
        with open(file, 'rb') as f:
            return pickle.load(f)


if __name__ == '__main__':
    solver = "ANT"
    model = Model()
    read_csv_file(model, "data/现实问题/demand.csv", "data/现实问题/school.csv")
    optimizer = model.TSP(solver)
    optimizer.run(model, no=18)
    Model.save_obj(model, f'现实问题/model-{solver}')




