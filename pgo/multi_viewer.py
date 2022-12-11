#!/usr/bin/python3.6
# -*- coding: UTF-8 -*-
from multiprocessing import Process, Queue

import pangolin as pango
import numpy as np
import OpenGL.GL as gl

from multi_robot_tools import MultiRobotTools

class MultiViewer3D(object):
  '''
  3d viewer for g2o maps
    - based off ficiciSLAM's viewer
       - github.com/kemfic/ficiciSLAM
  '''
  is_optim = False
  tform = np.array([[0.0, 0.0, 1.0, 0.0],
                    [1.0, 0.0, 0.0, 0.0],
                    [0.0, 1.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 1.0]])
  def __init__(self, graph,robot_num):
    self.init()
    self.color_init()
    self.graph = graph
    self.nodes = np.dot(graph.nodes, self.tform)
    self.edges = np.array(graph.edges)
    self.robot_num = robot_num
    self.nodes_keys = np.array(graph.nodes_keys)
    self.edges_key_pairs = np.array(graph.edges_key_pairs)
    self.multi_robot_tools = MultiRobotTools()
    self.separator_edges = self.graph.separator_edges 
    self.separator_nodes = np.dot(self.graph.separator_nodes,self.tform)
    
    self.partition_graph_np()
    while not pango.ShouldQuit():
      self.refresh()

  def init(self):
    w, h = (1024,768)
    f = 2000 #420

    pango.CreateWindowAndBind("g2o_stuff", w, h)
    gl.glEnable(gl.GL_DEPTH_TEST)

    # Projection and ModelView Matrices
    self.scam = pango.OpenGlRenderState(
        pango.ProjectionMatrix(w, h, f, f, w //2, h//2, 0.1, 100000),
        pango.ModelViewLookAt(0, -50.0, -10.0,
                              0.0, 0.0, 0.0,
                              0.0, -1.0, 0.0))#pango.AxisDirection.AxisY))
    self.handler = pango.Handler3D(self.scam)

    # Interactive View in Window
    self.dcam = pango.CreateDisplay()
    self.dcam.SetBounds(0.0, 1.0, 0.0, 1.0, -w/h)
    self.dcam.SetHandler(self.handler)
    self.dcam.Activate()

    pango.RegisterKeyPressCallback(ord('r'), self.optimize_callback)
    pango.RegisterKeyPressCallback(ord('t'), self.switch_callback)

  def color_init(self):
    color_list = [[255,69,0],[255,215,0],[0,255,127],[0,191,255],[138,43,226]]
    self.color_list = color_list
    self.separator_edge_color = [244,164,96]
    self.separator_node_color = [255,250,205]

  def refresh(self):
    #clear and activate screen
    gl.glClear(gl.GL_COLOR_BUFFER_BIT | gl.GL_DEPTH_BUFFER_BIT)
    gl.glClearColor(0.15, 0.15, 0.15, 0.0)
    #gl.glClearColor(1.0, 1.0, 1.0, 0.0)

    self.dcam.Activate(self.scam)
    if len(self.separator_nodes) >1:
      gl.glLineWidth(2)
      gl.glColor3f(self.separator_node_color[0]/255.0, self.separator_node_color[1]/255.0, self.separator_node_color[2]/255.0)
      pango.DrawCameras(self.separator_nodes)
    if len(self.separator_edges) >1:
      gl.glLineWidth(3)
      gl.glColor3f(self.separator_edge_color[0]/255.0, self.separator_edge_color[1]/255.0, self.separator_edge_color[2]/255.0)
      pango.DrawLines(self.separator_edges[:,0,:-1, -1], self.separator_edges[:,1,:-1,-1])

    for robot_id in range(self.robot_num):
      edge_color = self.color_list[robot_id]
      nodes = self.nodes_dict[robot_id]
      edges = self.edges_dict[robot_id]

      # render
      gl.glLineWidth(1)
      # render cameras
      if len(nodes) > 1:
        gl.glColor3f(edge_color[0]/255.0, edge_color[1]/255.0, edge_color[2]/255.0)
        # gl.glColor3f(1.0, 1.0, 1.0)

        pango.DrawCameras(nodes)
      # render edges
      if len(edges) > 1:
        gl.glColor3f(edge_color[0]/255.0, edge_color[1]/255.0, edge_color[2]/255.0)
        pango.DrawLines(edges[:,0,:-1, -1], edges[:,1,:-1,-1])

   
    pango.FinishFrame()

  def partition_graph_np(self):
    self.nodes_dict = {}
    self.edges_dict = {}

    for robot_id in range(self.robot_num):
      node_id_mask = np.array([self.multi_robot_tools.key2robot_id_g2o(key)==robot_id for key in self.nodes_keys])
      # node_id_mask = np.array([self.multi_robot_tools.key2robot_id_g2o(key) for key in self.nodes_keys])
      # node_id_mask = np.array([key for key in self.nodes_keys])

      # print(node_id_mask)
      
      self.nodes_dict[robot_id] = self.nodes[node_id_mask]

      node_key = self.nodes_keys[node_id_mask]

      # print("----------node_key---------------")
      # print("----------robot"+str(robot_id)+'-----------')
      # print(node_key)


      edge_id_mask = np.array([ (self.multi_robot_tools.key2robot_id_g2o(key_pair[0]) ==robot_id or self.multi_robot_tools.key2robot_id_g2o(key_pair[1])==robot_id)
                               for key_pair in self.edges_key_pairs])

      edges_key_pairs = self.edges_key_pairs[edge_id_mask]

      # print("----------edge_key---------------")
      # print("----------robot"+str(robot_id)+'-----------')
      # print(edges_key_pairs)

      self.edges_dict[robot_id] = self.edges[edge_id_mask]


  def update(self, graph=None):
    '''
    add new stuff to queues
    '''

    if self.q is None:
      return


    self.nodes = np.dot(graph.nodes, self.tform)
    self.edges = graph.edges
  def optimize_callback(self):
    self.graph.optimize()
    self.is_optim = False
    self.switch_callback()
  def switch_callback(self):
    self.is_optim = ~self.is_optim
    if self.is_optim:
      print("optimized")
      self.nodes = np.dot(self.graph.nodes_optimized, self.tform)
      self.edges = self.graph.edges_optimized
    else:
      print("original")
      self.nodes = np.dot(self.graph.nodes, self.tform)
      self.edges = self.graph.edges
