
from pgo.posegraph import PoseGraphSE3

base_dir = "/home/jiangpin/dataset/new_4robots/"

i=1
for i in range(4):
  file_name = base_dir+str(i)+'_separator.g2o'
  graph = PoseGraphSE3()
  print(graph)
  graph.load_file(file_name)
  print("loaded"+file_name)
  # graph.init_separator()

  # gfile = "/home/jiangpin/dataset/example_4robots/full_graph_renamed.g2o"
  # graph = PoseGraphSE3()
  # graph.load_file(gfile)
  #graph.optimize()
  print("loaded")
 