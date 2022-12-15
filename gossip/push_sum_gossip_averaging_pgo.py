"""
Push Sum Gossip Averaging class for parallel averaging using column stochastic mixing.

:author: Mido Assran
:description: Distributed averaging using column stochastic mixing.
              Based on the paper (kempe2003Gossip-based)
"""

import random
import time
import sys
sys.path.append("..")

import warnings

from mpi4py import MPI
import numpy as np

from .gossip_comm import GossipComm
from .pgo.se3_average import se3_average_at


# Message passing and network variables
COMM = GossipComm.comm
SIZE = GossipComm.size
UID = GossipComm.uid
NAME = GossipComm.name

# Default values
DEFAULT_NUM_GOSSIP_ITR = 1
DEFAULT_GOSSIP_TIME = 0.1 # time in seconds
TIMEOUT_PERIOD = 60e6 # maximum time in seconds to wait for messages (only valid in synch nets.)

class PushSumGossipAveragerPgo(object):
    """
    Distributed column stochastic averaging for pose graph optimization.

    :param average: Estimate of the network-wide average (updated at each itr.)
    :param synch: Whether to run the alg. synchronously (or asynchronously)
    :param terminate_by_time: Whether to terminate the alg. after a threshold time (itr. by default)
    :param termination_condition: Itr. count by default, otherwise threshold time (seconds)
    :param log: Whether to log the alg. variables at each iteration
    :param peers: UniqueIDs of neighbouring peers in net. (used for comm.)
    :param out_degree: Num. of rand. peers to choose/communicate with at each itr. (all by default)
    :param in_degree: Num. messages to expect at each itr. (only use in static synchronous nets.)
    :param all_reduce: Whether to perform All Reduce Averaging rather than gossip.
    """


    def __init__(self, synch=True,
                 peers=None,
                 terminate_by_time=False,
                 termination_condition=None,
                 log=False,
                 out_degree=None,
                 in_degree=SIZE,
                 all_reduce=False):
        """ Initialize the distributed averaging settings """

        # Break on all numpy warnings
        np.seterr(all='raise')

        self.average = None

        self.synch = synch

        # Set peers to all nodes in the network if we are not told who our peers are
        if not peers:
            peers = [i for i in range(SIZE) if i != UID]
        self.peers = peers
        # print(self.peers)

        self.terminate_by_time = terminate_by_time

        # Set the termination condition to the class defaults if not specified
        if termination_condition is None:
            if terminate_by_time:
                self.termination_condition = DEFAULT_GOSSIP_TIME
            else:
                self.termination_condition = DEFAULT_NUM_GOSSIP_ITR
        else:
            self.termination_condition = termination_condition

        self.log = log

        # Gossip with all peers by default
        if out_degree is None:
            self.out_degree = len(self.peers)
        else:
            self.out_degree = min(out_degree, len(self.peers))

        self.in_degree = in_degree

        self.all_reduce = all_reduce
        # print(self.out_degree)
        if (self.synch is True) and (self.terminate_by_time is True):
            warnings.warn("Use of synchronous gossip w/ time term. cond. will result in deadlocks.")

        if self.all_reduce is True:
            assert synch is True, "Cannot perform All Reduce asynchronously"
            assert self.log is False, "Cannot log All Reduce"


    def choose_gossip_peers(self):
        """ Choose a set of peers to gossip with from the set of all peers. """
        return random.sample(self.peers, self.out_degree)

    def make_stochastic_weight_column(self):
        """ Creates a stochastic column of weights for the gossip mixing matrix. """
        column = {}
        lo_p = 1.0 / (self.out_degree + 1.0)
        out_p = [1.0 / (self.out_degree + 1.0) for _ in range(self.out_degree)]
        # lo_p = 0.8
        # out_p = [0.2 / self.out_degree for _ in range(self.out_degree)]
        column['lo_p'] = lo_p
        column['out_p'] = out_p
        return column

    def push_messages_to_peers(self, peers, consensus_column, ps_w, ps_n):
        """
        Send scaled push sum numerator and push sum weights to peers.

        :type peers: list[int]
        :type consensus_column: list[float]
        :type ps_w: float
        :type ps_n: float
        :rtype: void
        """
        ps_n = ps_n.reshape([1,-1])
        for i, peer_uid in enumerate(peers):
            # push_message = np.append(consensus_column[i]*ps_n, consensus_column[i]*ps_w)
            # print('prepare')
            # print(ps_n)
            # print(consensus_column[i],ps_w)
            # push_message = np.append(ps_n,consensus_column[i],ps_w)
            # push_message = {'ps_n':ps_n.shape,'col_w':consensus_column[i],'ps_w':ps_w}
            # ps_n = 
            # push_message = COMM.scatter(push_message,root=0)
            # print(push_message)
            # print(type(push_message))
            # push_message = []

            # print('-------'+str(peer_uid)+'------')
            # print(ps_n.shape)
            # print(ps_w,col_w)
            # push_message = np.append(ps_n,ps_w,consensus_column[i])
            push_message = np.append(ps_n,float(ps_w))
            push_message = np.append(push_message,consensus_column[i])
            
            # print("message use to push")
            # print('col_w',push_message[-1])
            # print('ps_w',push_message[-2])
            # print(push_message.shape)
            # print('finished')
            # print(push_message)
            _ = COMM.Ibsend(push_message, dest=peer_uid)

    def recieve_asynchronously(self):
        """
        Probe buffer (non-blocking) & and retrieve all messages until the receive buffer is empty.

        :rtype: dict('num_messages': int, 'ps_w': float, 'ps_n': np.array[float] or float)
        """
        itr = 0
        ps_w_list = []
        ps_n_list = []
        gs_w_list = []
        info = MPI.Status()

        while COMM.Iprobe(source=MPI.ANY_SOURCE, status=info):

            itr += 1
            # Receive message
            data = np.empty(self.average.size +2+1, dtype=np.float64)
            COMM.Recv(data, info.source)

            ps_w_list.append(data[-1])
            gs_w_list.append(data[-1-self.out_degree:-1])


            # This is some logic to allow handling of both gossiped constant values and vectors
            try:
                ps_n_list.append(data[:-1-self.out_degree].reshape(self.average.shape))
            except AttributeError:
                # ps_n += data[0]
                print(Exception)

            info = MPI.Status()

        return {'num_messages': itr, 'ps_w': ps_w_list, 'ps_n': ps_n_list,'gs_w':gs_w_list}

    def receive_synchronously(self):
        """
        Probe buffer (blocking w/ timeout protection) and retrieve iteration messages.

        :rtype: dict('num_messages': int, 'ps_w': float, 'ps_n': np.array[float] or float)
        """
        ps_w = 0
        
        # ps_w_list = []
        ps_n_list = []
        col_w_list = []

        itr = 0

        for _ in range(self.in_degree):

            info = MPI.Status()

            # This is a modification to the synchronous logic to avoid deadlocks by using timeouts
            start_time = time.time()
            threshold = start_time + TIMEOUT_PERIOD
            break_flag = False
            while COMM.Iprobe(source=MPI.ANY_SOURCE, status=info) is False:
                if time.time() > threshold:
                    break_flag = True
                    break
            if break_flag:
                break

            # Receive message
            # @todo in degree or out degree? 
            data = np.empty(self.average.size +2, dtype=np.float64)
            COMM.Recv(data, info.source)
            # print("----recieved data----")
            # print(data.shape)
            
            col_w = data[-1]
            ps_w_in = data[-2]
            # print(col_w,ps_w_in)
            ps_n = data[0:-2].reshape([-1,6])
            # Update if received a message
            itr += 1
            
            # print("curr iter"+str(itr))

            ps_w += col_w*ps_w_in
            # print('ps_w',ps_w,'ps_w_in',ps_w_in)
            #default out
            col_w_list.append(col_w)

            # This logic allows handling of gossiped constant values, vectors, and matrices
            try:
                # ps_n_list.append(data[:-1-self.out_degree].reshape(self.average.shape))
                ps_n_list.append(ps_n)
            except AttributeError:
                print(Exception)


            del info
        ps_n_list = np.array(ps_n_list)
        col_w_list = np.array(col_w_list)
        # print(ps_n_list.shape)
        # print(col_w_list.shape)
        return {'num_messages': itr, 'ps_w': ps_w, 'ps_n': ps_n_list,'col_w':col_w_list}


    def gossip(self, gossip_value, ps_weight=1.0, just_probe=False, start_together=False):
        """
        Perform the distribued gossip averaging (settings given by the instance parameters).

        :type gossip_value: float
        :type ps_weight: float
        :type just_probe: Boolean
        :type start_together: Boolean
        :rtype:
               log is True: dict('avg': GossipLogger,
                                 'ps_n': GossipLogger,
                                 'ps_w': GossipLogger)

               log is False: dict('avg': float,
                                  'ps_n': float,
                                  'ps_w': float,
                                  'rcvd_flag': Boolean)
        """

        if (self.synch is True) and (self.terminate_by_time is True):
            warnings.warn("Use of synchronous gossip w/ time term. cond. will result in deadlocks.")

        gossip_value = np.array(gossip_value, dtype=np.float64)

        # Initialize push sum gossip
        self.average = gossip_value
        log = self.log
        ps_n = gossip_value # push sum numerator
        ps_w = ps_weight # push sum weight
        avg = ps_n / ps_w # push sum estimate
        itr = 0

        # All reduce (NOT GOSSIP)
        if self.all_reduce is True:
            assert self.synch is True, "Cannot perform All Reduce asynchronously"
            assert self.log is False, "Cannot log All Reduce"
            total = np.empty(self.average.size, dtype=np.float64)
            COMM.Allreduce(gossip_value, total, op=MPI.SUM)
            avg = total / SIZE
            return {"avg": avg, "ps_w" : ps_weight, "ps_n": avg, "rcvd_flag": True}

        if self.out_degree == len(self.peers):
            static_peers = True
            peers = self.peers
        else:
            static_peers = False

        if log:
            from .gossip_log import GossipLog
            l_avg = GossipLog()
            l_ps_w = GossipLog()
            l_ps_n = GossipLog()
            l_avg.log(avg, itr)
            l_ps_w.log(ps_w, itr)
            l_ps_n.log(ps_n, itr)

        if self.terminate_by_time is False:
            num_gossip_itr = self.termination_condition
            condition = itr < num_gossip_itr
        else:
            gossip_time = self.termination_condition
            end_time = time.time() + gossip_time
            condition = time.time() < end_time

        # Goes high when node receives at least one message during any iteration (for external use)
        ext_rcvd_flag = False
        # Goes high when a gossip should be performed (when it has changed)
        rcvd_flag = True

        # Start averaging together if that's what the caller desires
        if start_together is True:
            COMM.Barrier()

        # Gossip loop
        while condition:

            if self.synch is True:
                COMM.Barrier()

            # Update iteration
            itr += 1

            column = self.make_stochastic_weight_column()
            out_p = column['out_p'] # vector
            lo_p = column['lo_p']   # scalar
            lo_w = ps_w * lo_p
            # lo_n = ps_n


            # Conditions for numerical instability prevention
            if (lo_w > 1e-5) and (just_probe is False) and (rcvd_flag is True):
                if static_peers is False:
                    peers = self.choose_gossip_peers()
                self.push_messages_to_peers(peers, out_p, ps_w, ps_n)
            else:
                lo_w = ps_w
                # lo_n = ps_n

            if self.synch is False:
                rcvd = self.recieve_asynchronously()
            else:
                rcvd = self.receive_synchronously()

            ps_n = se3_average_at(rcvd['ps_n'],rcvd['col_w'],ps_n,lo_p)
            ps_w = lo_w + rcvd['ps_w']
            # ps_n = lo_n + rcvd['ps_n']

            # If received at least one message, set external rcvd flag to high
            if rcvd['num_messages'] > 0:
                ext_rcvd_flag = True
                rcvd_flag = True
            else:
                rcvd_flag = False

            avg = ps_n / ps_w
            if log is True:
                l_avg.log(avg, itr)
                l_ps_w.log(ps_w, itr)
                l_ps_n.log(ps_n, itr)

            if self.terminate_by_time is False:
                condition = itr < num_gossip_itr
            else:
                condition = time.time() < end_time

        self.average = avg

        if log is True:
            return {"avg": l_avg, "ps_w": l_ps_w, "ps_n": l_ps_n}
        else:
            return {"avg": avg, "ps_w" : ps_w, "ps_n": ps_n, "rcvd_flag": ext_rcvd_flag}


if __name__ == "__main__":

    def demo(gossip_value):
        """
        Demo of the use of the PushSumGossipAverager class.

        To run the demo, run the following form the command line:
            mpiexec -n $(num_nodes) python -m push_sum_gossip_averaging
        """

        # Initialize averager
        psga = PushSumGossipAveragerPgo(synch=True,
                                     peers=[(UID + 1) % SIZE, (UID + 2) % SIZE],
                                     terminate_by_time=False,
                                     termination_condition=100,
                                     log=True,
                                     in_degree=2)

        loggers = psga.gossip(gossip_value)
        l_avg = loggers["avg"] # Average logger

        # Print this node's most recently logged values for the push sum variables
        l_avg.print_gossip_value(UID)

    # Run a demo where nodes average their unique IDs
    demo(gossip_value=UID)
