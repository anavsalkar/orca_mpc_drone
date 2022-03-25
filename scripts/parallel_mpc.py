#!/home/atharva/BTP/btp/bin/python

# importing the mp module
import multiprocessing as mp 
import os
from main import *
import rospy

def worker1(my_ID, n_agents,end_pos):
	# printing process id
	# print("ID of process running worker1: {}".format(os.getpid()))
    mpc(my_ID,n_agents)




if __name__ == "__main__":
	
	# printing main program process id
	print("ID of main process: {}".format(os.getpid()))

	n_agents = 6

	# creating processes # args=(my_ID, n_agents, [final pos])
	p1 = mp.Process(target=mpc, args=(0,6,[-3,-5,3])) 
	p2 = mp.Process(target=mpc, args=(1,6,[0,-5,3])) 
	p3 = mp.Process(target=mpc, args=(2,6,[3,-5,3])) 
	p4 = mp.Process(target=mpc, args=(3,6,[3,5,3])) 
	p5 = mp.Process(target=mpc, args=(4,6,[0,5,3])) 
	p6 = mp.Process(target=mpc, args=(5,6,[-3,5,3])) 

	# # creating processes # args=(my_ID, n_agents, [final pos])
	# p1 = mp.Process(target=mpc, args=(0,6,[4,-6,3])) 
	# p2 = mp.Process(target=mpc, args=(1,6,[0,-6,3])) 
	# p3 = mp.Process(target=mpc, args=(2,6,[-4,-6,3])) 
	# p4 = mp.Process(target=mpc, args=(3,6,[-4,6,3])) 
	# p5 = mp.Process(target=mpc, args=(4,6,[0,6,3])) 
	# p6 = mp.Process(target=mpc, args=(5,6,[4,6,3])) 

	# # creating processes # args=(my_ID, n_agents, [final pos])
	# p1 = mp.Process(target=mpc, args=(0,n_agents,[2.5,-4.33,3])) 
	# p2 = mp.Process(target=mpc, args=(1,n_agents,[-2.4,-4.33,3])) 
	# p3 = mp.Process(target=mpc, args=(2,n_agents,[-5,0,3])) 
	# p4 = mp.Process(target=mpc, args=(3,n_agents,[-2.5,4.33,3])) 
	# p5 = mp.Process(target=mpc, args=(4,n_agents,[2.5,4.33,3])) 
	# p6 = mp.Process(target=mpc, args=(5,n_agents,[5,0,3])) 

	# # creating processes # args=(my_ID, n_agents, [final pos])
	# p1 = mp.Process(target=mpc, args=(0,n_agents,[10,-10,3])) 
	# p2 = mp.Process(target=mpc, args=(1,n_agents,[-10,-10,3])) 
	# p3 = mp.Process(target=mpc, args=(2,n_agents,[-10,10,3])) 
	# p4 = mp.Process(target=mpc, args=(3,n_agents,[10,10,3])) 

	# # creating processes # args=(my_ID, n_agents, [final pos])
	# p1 = mp.Process(target=mpc, args=(0,6,[0,10,3])) 
	# p2 = mp.Process(target=mpc, args=(1,6,[10,10,3])) 
	# p3 = mp.Process(target=mpc, args=(2,6,[-10,10,3])) 
	# p4 = mp.Process(target=mpc, args=(3,6,[10,0,3])) 
	# p5 = mp.Process(target=mpc, args=(4,6,[-10,0,3])) 
	# p6 = mp.Process(target=mpc, args=(5,6,[0,0,3])) 

	# # creating processes # args=(my_ID, n_agents, [final pos])
	# p1 = mp.Process(target=mpc, args=(0,n_agents,[2,10,3])) 
	# p2 = mp.Process(target=mpc, args=(1,n_agents,[6,5,3])) 
	# p3 = mp.Process(target=mpc, args=(2,n_agents,[10,0,3])) 
	# p4 = mp.Process(target=mpc, args=(3,n_agents,[-10,0,3])) 
	# p5 = mp.Process(target=mpc, args=(4,n_agents,[-6,5,3])) 
	# p6 = mp.Process(target=mpc, args=(5,n_agents,[-2,10,3])) 


	# ## 12 agents
	# # creating processes # args=(my_ID, n_agents, [final pos])
	# p1 = mp.Process(target=mpc, args=(0,n_agents,[-10,0,3])) 
	# p2 = mp.Process(target=mpc, args=(1,n_agents,[-9,5,3])) 
	# p3 = mp.Process(target=mpc, args=(2,n_agents,[-5,9,3])) 
	# p4 = mp.Process(target=mpc, args=(3,n_agents,[0,10,3])) 
	# p5 = mp.Process(target=mpc, args=(4,n_agents,[5,9,3])) 
	# p6 = mp.Process(target=mpc, args=(5,n_agents,[9,5,3])) 
	# # p7 = mp.Process(target=mpc, args=(6,n_agents,[10,0,3])) 
	# # p8 = mp.Process(target=mpc, args=(7,n_agents,[9,-5,3])) 
	# # p9 = mp.Process(target=mpc, args=(8,n_agents,[5,-9,3])) 
	# # p10 = mp.Process(target=mpc, args=(9,n_agents,[0,-10,3])) 
	# # p11 = mp.Process(target=mpc, args=(10,n_agents,[-5,-9,3])) 
	# # p12 = mp.Process(target=mpc, args=(11,n_agents,[-9,-5,3]))
	
	# starting processes
	p1.start()
	p2.start()
	p3.start()
	p4.start()
	p5.start()
	p6.start()
	# p7.start()
	# p8.start()
	# p9.start()
	# p10.start()
	# p11.start()
	# p12.start()

	rospy.init_node('talker', anonymous=True)

	# # process IDs
	# print("ID of process p1: {}".format(p1.pid))
	# print("ID of process p2: {}".format(p2.pid))

	# wait until processes are finished
	p1.join()
	p2.join()
	p3.join()
	p4.join()
	p5.join()
	p6.join()
	# p7.join()
	# p8.join()
	# p9.join()
	# p10.join()
	# p11.join()
	# p12.join()

	# both processes finished
	print("Both processes finished execution!")

	# check if processes are alive
	print("Process p1 is alive: {}".format(p1.is_alive()))
	print("Process p2 is alive: {}".format(p2.is_alive()))



