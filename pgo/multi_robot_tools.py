#!/usr/bin/python3.6
# -*- coding: UTF-8 -*-

import os
import copy

import numpy as np
from g2o_tool import G2oTool

class Separator():
    def __init__(self) -> None:
        self.vertex = {}
        self.edge = {}
        
    def exists(self,key_pair):
        for sep_key_pair in self.edge.keys():
            if(sep_key_pair[0] == key_pair[0] and sep_key_pair[1]==key_pair[1]):
                return True
            if(sep_key_pair[0] ==key_pair[1] and sep_key_pair[1]==key_pair[0]):
                return True
        return False
    
    def add(self,key_pair,vertex0,vertex1,edge):
        self.vertex[key_pair[0]] = vertex0
        self.vertex[key_pair[1]] = vertex1
        self.edge[key_pair] = edge


class MultiRobotTools():
    def __init__(self,data_dir=None,robot_num=0) -> None:
        self.data_dir = data_dir
        self.robot_num = robot_num
        self.vertex_dict = {}
        self.edge_dict = {}
        #sum graph
        self.vertex_dict_sum = {}
        self.edge_dict_sum = {}
        #seprarator
        self.separator = Separator()
        self.g2o_tool = G2oTool()
    
    def read_g2o(self):
        for robot_id in range(self.robot_num):
            file_name = os.path.join(self.data_dir,str(robot_id)+'.g2o')
            vertex,edge = self.g2o_tool.read(file_name)
            # print(edge)
            self.vertex_dict[robot_id] = vertex
            self.edge_dict[robot_id] = edge

    def rename_gtsam_id(self):

        edge_dict_copy = copy.deepcopy(self.edge_dict)
        vertex_dict_copy = copy.deepcopy(self.vertex_dict)
        vertex_rename_sum = {}
        edge_rename_sum = {}
        for robot_id in range(self.robot_num):
            vertex_dict = vertex_dict_copy[robot_id]
            edge_dict = edge_dict_copy[robot_id]
            # print(robot_id)
            # print(vertex_dict.keys())
            # print(edge_dict.keys())
            new_vertex_dict = {}
            for key in vertex_dict.keys():
                newkey = self.id_gtsam2g2o(key)
                new_vertex_dict[newkey] = vertex_dict[key]

            # vertex_dict[newkey] = vertex_dict.pop(key)
            new_edge_dict = {}

            for key_pair in edge_dict.keys():
                new_key0 = self.id_gtsam2g2o(key_pair[0])
                new_key1 = self.id_gtsam2g2o(key_pair[1])
                new_edge_dict[(new_key0,new_key1)] = edge_dict[key_pair]
                # edge_dict[(new_key0,new_key1)] = edge_dict.pop(key_pair)
            vertex_rename_sum.update(new_vertex_dict)
            edge_rename_sum.update(new_edge_dict)

            file_name = os.path.join(self.data_dir,str(robot_id)+'_renamed.g2o')
            self.g2o_tool.write_dict(file_name,new_vertex_dict,new_edge_dict)
        self.aggregate_graph()
        file_name = os.path.join(self.data_dir,'full_graph_renamed.g2o')
        self.g2o_tool.write_dict(file_name,vertex_rename_sum,edge_rename_sum) 

    def is_separator(self,key_pair):
        key0 = key_pair[0]
        key1 = key_pair[1]
        id0  = self.key2robot_id(key0)
        id1  = self.key2robot_id(key1)
        return id0 != id1

    def is_separator_g2o(self,key_pair):
        key0 = key_pair[0]
        key1 = key_pair[1]
        id0  = self.key2robot_id_g2o(key0)
        id1  = self.key2robot_id_g2o(key1)
        return id0!=id1

    def key2robot_id(self,key):
        robot_id = int(key)//100000000000000000
        robot_id -= 69
        return robot_id

    def key2robot_id_g2o(self,key):
        robot_id = int(key)//1000000
        robot_id-=1
        return robot_id

    def id_gtsam2g2o(self,key):
        #rename the id in gtsam so that g2o can handle
        # (id in gtsam is too long)
        key = int(key)
        robot_id = self.key2robot_id(key)
        result =key//1000000
        #10^6
        robot_id+=1
        new_key = robot_id*1000000+key-result*1000000
        return str(new_key)
    
    def aggregate_graph(self):
        #aggregate graph from each g2o file into one g2o file
        for robot_id in range(self.robot_num):
            vertex_dict = self.vertex_dict[robot_id]
            edge_dict = self.edge_dict[robot_id]

            self.vertex_dict_sum.update(vertex_dict)
            self.edge_dict_sum.update(edge_dict)
        file_name = os.path.join(self.data_dir,'full_graph'+'.g2o')
        self.g2o_tool.write_dict(file_name,self.vertex_dict_sum,self.edge_dict_sum)


    def partition_graph(self,file_name):
        vertex,edge = self.g2o_tool.read(file_name)
        base_name = file_name.split('.')

        vertex_dict = {}
        edge_dict = {}
        
        for robot_id in range(self.robot_num):
            for key in vertex.keys():
                if(self.key2robot_id(key)==robot_id):
                    vertex_dict[robot_id].update({key,vertex[key]})
            for key_pair in edge.keys():
                if(self.key2robot_id(key_pair[0])==robot_id or self.key2robot_id(key_pair[1])==robot_id):
                    edge_dict[robot_id].update({key_pair,edge[key_pair]})
            self.g2o_tool.write_dict(base_name+'_'+str(robot_id)+'.g2o',vertex_dict[robot_id],edge_dict[robot_id])

    def aggregate_separator(self):
        for each_robot in self.edge_dict.keys():
            edge_dict = self.edge_dict[each_robot]
            for key_pair in edge_dict.keys():
                if(self.is_separator(key_pair)):
                    if(not self.separator.exists(key_pair)):
                        id0 = self.key2robot_id(key_pair[0])
                        id1 = self.key2robot_id(key_pair[1])
                        
                        vertex0 = self.vertex_dict[id0][key_pair[0]]
                        vertex1 = self.vertex_dict[id1][key_pair[1]]
                        edge = edge_dict[key_pair]
                        self.separator.add(key_pair,vertex0,vertex1,edge)
        file_name = os.path.join(self.data_dir,'separator'+'.g2o')
        self.g2o_tool.write_dict(file_name,self.separator.vertex,self.separator.edge)

    def spread_separator(self):
        #spread separator among robot for dist-optimization
        self.aggregate_separator()
        edge_dict_copy = copy.deepcopy(self.edge_dict)
        vertex_dict_copy = copy.deepcopy(self.vertex_dict)
        for robot_id in edge_dict_copy.keys():
            edge_dict = edge_dict_copy[robot_id]
            vertex_dict = vertex_dict_copy[robot_id]
            # print(self.separator.edge.keys())
            edge_dict.update(self.separator.edge)
            vertex_dict.update(self.separator.vertex)
            file_name = os.path.join(self.data_dir,str(robot_id)+'_separator'+'.g2o')
            self.g2o_tool.write_dict(file_name,vertex_dict,edge_dict)

    def graph_info(self):
        print("----Total info---")
        print("total vertex num:"+str(len(self.vertex_dict_sum)))
        print("total edge num:"+str(len(self.edge_dict_sum)))
        print("---Separator info---")
        print("separator vertex num:"+str(len(self.separator.vertex)))
        print("separator edge num:"+str(len(self.separator.edge)))



if __name__ == '__main__':
    # data_dir = '/home/jiangpin/dataset/test_4robots'
    data_dir = "/home/jiangpin/dataset/example_4robots/"
    num = 4
    multi_robot_tools = MultiRobotTools(data_dir,num)
    multi_robot_tools.read_g2o()
    
    # multi_robot_tools.spread_separator()
    # multi_robot_tools.aggregate_graph()
    # multi_robot_tools.graph_info()
    multi_robot_tools.rename_gtsam_id()