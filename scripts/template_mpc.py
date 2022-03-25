#
#   This file is part of do-mpc
#
#   do-mpc: An environment for the easy, modular and efficient implementation of
#        robust nonlinear model predictive control
#
#   Copyright (c) 2014-2019 Sergio Lucia, Alexandru Tatulea-Codrean
#                        TU Dortmund. All rights reserved
#
#   do-mpc is free software: you can redistribute it and/or modify
#   it under the terms of the GNU Lesser General Public License as
#   published by the Free Software Foundation, either version 3
#   of the License, or (at your option) any later version.
#
#   do-mpc is distributed in the hope that it will be useful,
#   but WITHOUT ANY WARRANTY; without even the implied warranty of
#   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#   GNU Lesser General Public License for more details.
#
#   You should have received a copy of the GNU General Public License
#   along with do-mpc.  If not, see <http://www.gnu.org/licenses/>.

# from typing_extensions import final
import numpy as np
from numpy.linalg import norm
from casadi import *
from casadi.tools import *
import pdb
import sys
sys.path.append('../../')
import do_mpc
# from main import *
import settings


def template_mpc(model,my_ID,n_agents,final_pos):
    """
    --------------------------------------------------------------------------
    template_mpc: tuning parameters
    --------------------------------------------------------------------------
    """
    mpc = do_mpc.controller.MPC(model)
    n_horizon = 20
    delT = 0.1

    setup_mpc = {
        'n_robust': 0,
        'n_horizon': n_horizon,
        't_step': delT,
        'state_discretization': 'discrete',
        'store_full_solution':True,
    }

    mpc.set_param(**setup_mpc)
    mpc.set_param(nlpsol_opts = {'ipopt.linear_solver': 'MA27'})
    suppress_ipopt = {'ipopt.print_level':0, 'ipopt.sb': 'yes', 'print_time':0}
    mpc.set_param(nlpsol_opts = suppress_ipopt)
    
    

    # if my_ID!=1:
    #     suppress_ipopt = {'ipopt.print_level':0, 'ipopt.sb': 'yes', 'print_time':0}
    #     mpc.set_param(nlpsol_opts = suppress_ipopt)
    
    X = model.x['x', 0]
    Y = model.x['x', 1]
    Z = model.x['x', 2]
    X_dot = model.x['x', 3]
    Y_dot = model.x['x', 4]
    Z_dot = model.x['x', 5]
    Phi = model.x['x', 6]
    Theta = model.x['x', 7]
    Psi = model.x['x', 8]
    Phi_dot = model.x['x', 9]
    Theta_dot = model.x['x',10]
    Psi_dot = model.x['x', 11]
    
    _x = model.x['x']
    _u = model.u['u']
    #_lambda = model.u['lambda']
    
    # Q = np.zeros((12,12))
    # Q[0,0] = 1
    # Q[1,1] = 1
    # Q[2,2] = 1
    # Q[3,3] = 1
    # Q[4,4] = 1
    # Q[5,5] = 1
    # Q[6,6] = 5
    # Q[7,7] = 5
    # Q[8,8] = 5
    # Q[9,9] = 1
    # Q[10,10] = 1
    # Q[11,11] = 1
    
    # R = np.zeros((4,4))
    # R[0,0] = 0.1
    # R[1,1] = 5
    # R[2,2] = 5
    # R[3,3] = 5
    
    
    Q = np.zeros((12,12))
    Q[0,0] = 1
    Q[1,1] = 1
    Q[2,2] = 1
    Q[3,3] = 1
    Q[4,4] = 1
    Q[5,5] = 1
    Q[6,6] = 5
    Q[7,7] = 5
    Q[8,8] = 5
    Q[9,9] = 1
    Q[10,10] = 1
    Q[11,11] = 1
    
    R = np.zeros((4,4))
    R[0,0] = 5
    R[1,1] = 5
    R[2,2] = 5
    R[3,3] = 5
    
    
    
    
    x_ref = np.zeros((12,1))
    x_ref[2] = final_pos[2]
    x_ref[1] = final_pos[1]
    x_ref[0] = final_pos[0]
    err_x = _x - x_ref
    
    mterm = (err_x.T)@Q@err_x #+ (_u.T)@R@_u
    lterm = mterm + (_u.T)@R@_u
    #(_u.T)@R@_u
    
    # mterm = model.aux['cost']
    # lterm = model.aux['cost'] # terminal cost

    mpc.set_objective(mterm=mterm, lterm=lterm)
    mpc.set_rterm(u=1*np.ones((4,1)))

    max_x = np.array([[20.0], [20.0], [12.0], [0.75], [0.75], [0.5], [0.5], [0.5], [0.5], [0.5], [0.5], [0.5]])
    max_u = np.array([[5],[2],[2],[2]])
    #min_u = np.array([[-0.2],[-2],[2],[2]])

    mpc.bounds['lower','_x','x'] = -max_x
    mpc.bounds['upper','_x','x'] =  max_x
    

    mpc.bounds['lower','_u','u'] = -max_u
    mpc.bounds['upper','_u','u'] =  max_u
    


    tvp_temp = mpc.get_tvp_template()
    # get_Odom(model, my_ID, n_agents,OdomStruct)
    def get_Odom(model):
        from main import get_state, get_u
        # import settings
        # my_x = get_state(my_ID)
        my_x = settings.OdomStruct[my_ID]
        # print('my_x:'+str(my_ID))
        # print(my_x.T)
        for i in (j for j in range(n_agents) if j!= my_ID):
            x_var_name = 'x'
            # x_curr = get_state(i)
            x_curr = settings.OdomStruct[i]

            u_var_name = 'u_ORCA' + str(i+1)
            n_var_name = 'n_ORCA' + str(i+1)
            sign_var_name = 's_ORCA' + str(i+1)
            v_b_var_name = 'v_b' + str(i+1)

            delT = 0.1
            for k in range(n_horizon+1):
                # tvp_temp['_tvp',k,G_var_name]=G

                try: 
                    # _X = dummy_var
                    _X = mpc.data.prediction(('_x',x_var_name,0))
                    _Y = mpc.data.prediction(('_x',x_var_name,1))
                    _Z = mpc.data.prediction(('_x',x_var_name,2))
                    _VX = mpc.data.prediction(('_x',x_var_name,3))
                    _VY = mpc.data.prediction(('_x',x_var_name,4))
                    _VZ = mpc.data.prediction(('_x',x_var_name,5))

                    ## try changing the prediction time step
                    X = _X[0,k,0]
                    Y = _Y[0,k,0]
                    Z = _Z[0,k,0]
                    VX = _VX[0,k,0]
                    VY = _VY[0,k,0]
                    VZ = _VZ[0,k,0]
                    delT = 0.1
                    # print('prediction')
                    if X==0 and Y==0 and Z==0:
                        # pos = multi_agent_params['init_pos']
                        X = my_x[0] + delT*my_x[3]*k
                        Y = my_x[1] + delT*my_x[4]*k
                        Z = my_x[2] + delT*my_x[5]*k
                        VX = my_x[3]
                        VY = my_x[4]
                        VZ = my_x[5]
                        # print('cont')
                    
                    # print(k)
                except: 
                    X = my_x[0] + delT*my_x[3]*k
                    Y = my_x[1] + delT*my_x[4]*k
                    Z = my_x[2] + delT*my_x[5]*k
                    VX = my_x[3]
                    VY = my_x[4]
                    VZ = my_x[5]
                    # print('cont exception')
                try: 
                    p_a = np.array([[X[0]],[Y[0]]])
                    p_b = np.array([[x_curr[0][0]+k*x_curr[3][0]*delT],[x_curr[1][0]+k*x_curr[4][0]*delT]])
                    v_a = np.array([[VX[0]],[VY[0]]])
                    v_b = np.array([[x_curr[3][0]],[x_curr[4][0]]])
                    # print('no exception')
                    # print(x_curr)
                    # print(p_b)
                    # print(X)
                except:
                    p_a = np.array([[X],[Y]])
                    p_b = np.array([[x_curr[0][0]+k*x_curr[3][0]*delT],[x_curr[1][0]+k*x_curr[4][0]*delT]])
                    v_a = np.array([[VX],[VY]])
                    v_b = np.array([[x_curr[3][0]],[x_curr[4][0]]])
                    # print('exception')
                    # print(x_curr)
                    # print(X)
                    # print(p_b)
                r_a = 1
                r_b = 1
                tau = 3-0.1*k
                # tau = 2.5
                # print(k)
                # print(v_a.shape)


                sign  = 1
                u = np.array([[0.1],[0.5]])
                n = np.array([[0.1],[0.5]])

                if norm(p_b - p_a) < (r_a+r_b+0.1): 
                    # print(my_ID)
                    # print('actual p_b-p_a: ')
                    # print(norm(p_b - p_a))
                    extra = -((r_a+r_b - norm(p_b-p_a))/2)

                    # p_b = p_b + 0.3*(p_b - p_a)/norm(p_b - p_a)
                    # print('in exception')
                    if my_ID == 1:
                        print(norm(p_b - p_a))
                else:
                    extra = 0


                (u,n,sign) = get_u(p_a, p_b, v_a, v_b, r_a+extra, r_b+extra, tau)
                
                

                tvp_temp['_tvp',k,u_var_name] = u
                tvp_temp['_tvp',k,n_var_name] = n
                tvp_temp['_tvp',k,sign_var_name] = sign
                tvp_temp['_tvp',k,'v_a'] = v_a
                tvp_temp['_tvp',k,v_b_var_name] = v_b
            

        return tvp_temp

    # mpc.set_tvp_fun(get_Odom(model,my_ID,n_agents,OdomStruct))
    mpc.set_tvp_fun(get_Odom)

    for i in (j for j in range(n_agents) if j!= my_ID):
        u_var_name = 'u_ORCA' + str(i+1)
        n_var_name = 'n_ORCA' + str(i+1)
        sign_var_name = 's_ORCA' + str(i+1)
        u = model.tvp[u_var_name]
        n = model.tvp[n_var_name]
        s = model.tvp[sign_var_name]
        v_a = model.tvp['v_a']
        
        v_b_var_name = 'v_b' + str(i+1)
        v_b = model.tvp[v_b_var_name]

        v = np.array([[_x[3]],
                      [_x[4]]])
        # print(s)
        # print(v_a)
        # print(u)


        # ORCA_ab = s*(v - dot((v_a + 0.5*u),n))
        ORCA_ab = s*dot(((v) - (v_a + 0.5*u)),n)
        constraint_name = 'ORCA' + str(my_ID) + '|' + str(i)
        mpc.set_nl_cons(constraint_name, ORCA_ab, ub = 1e-2, soft_constraint=True, penalty_term_cons=10)

    mpc.setup()

    return mpc


