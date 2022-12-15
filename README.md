# gossip-pgo
gossip-based pose graph optimization

## run
mpiexec -n 4 python3 -m gossip.push_sum_gossip_pgo
mpiexec -n 4 python3 -m gossip.push_sum_gossip_gradient_descent