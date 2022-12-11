#!/usr/bin/python3.6
# -*- coding: UTF-8 -*-

class G2oTool:
    def __init__(self) -> None:
        self.vertex = {}
        self.edge = {}
    
    def read(self,file_name):
        vertex = {}
        edge = {}
        with open(file_name,'r') as g2o_file:
            for each_line in g2o_file:
                try:
                    g2o__line = each_line.split(' ')
                    identifier= g2o__line[0]
                    if identifier == 'VERTEX_SE3:QUAT':
                        vertex[g2o__line[1]] = (g2o__line[2:-1])
                    elif identifier=='EDGE_SE3:QUAT':
                        edge[(g2o__line[1],g2o__line[2])] = (g2o__line[3:-1])
                except ValueError:
                    pass
        
        return vertex,edge
    
    def read_edge_keys(self,file_name):
        edge_keys = []
        with open(file_name,'r') as g2o_file:
            for each_line in g2o_file:
                try:
                    g2o__line = each_line.split(' ')
                    identifier= g2o__line[0]
                    if identifier == 'VERTEX_SE3:QUAT':
                        pass
                    elif identifier=='EDGE_SE3:QUAT':
                        edge_keys.append([int(g2o__line[1]),int(g2o__line[2])])
                except ValueError:
                    pass
        return edge_keys

    def write(self,file_name,vertex_lines,edge_lines):
        with open(file_name,'w') as g2o_file:
            g2o_file.writelines(vertex_lines)
            g2o_file.writelines(edge_lines) 

    def write_dict(self,file_name,vertex_dict,edge_dict):
        vertex_lines = self.vertex_dict2lines(vertex_dict)
        edge_lines = self.edge_dict2lines(edge_dict)
        self.write(file_name,vertex_lines,edge_lines)

    def vertex_dict2list(self,vertex_dict):
        vertex_list = []
        for key,data in zip(vertex_dict.keys(),vertex_dict.values()):
            vertex_list.append(key)
            for each in data:
                vertex_list.append(each)
        return vertex_list

    def edge_dict2list(self,edge_dict):
        edge_list = []
        for key_pair,data in zip(edge_dict.keys(),edge_dict.values()):
            edge_list.append(key_pair[0])
            edge_list.append(key_pair[1])
            for each in data:
                edge_list.append(each)
        return edge_list
    
    def vertex_dict2lines(self,vertex_dict):
        vertex_lines = []
        for key,data in zip(vertex_dict.keys(),vertex_dict.values()):
            line = 'VERTEX_SE3:QUAT '
            line =line+key
            for each in data:
                line = line+' '+each
            line += '\n'
            vertex_lines.append(line)
        return vertex_lines

    def edge_dict2lines(self,edge_dict):
        edge_lines = []
        for key_pair,data in zip(edge_dict.keys(),edge_dict.values()):
            edge_line = 'EDGE_SE3:QUAT '
            
            edge_line += key_pair[0]+' '+key_pair[1]
            for each in data:
                edge_line = edge_line+' '+each
            edge_line += '\n'
            edge_lines.append(edge_line)
        return edge_lines

if __name__ == '__main__':
    g2o_tool = G2oTool()
    vertex,edge = g2o_tool.read('/home/jiangpin/dataset/example_4robots/0.g2o')
    edge_keys = g2o_tool.read_edge_keys('/home/jiangpin/dataset/example_4robots/0.g2o')

    # print(vertex)
    # print()
    print(edge_keys)