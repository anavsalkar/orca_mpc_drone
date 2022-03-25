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

import numpy as np
from casadi import *
from casadi.tools import *
import pdb
import sys
sys.path.append('../../')
import do_mpc


def template_simulator(model,my_ID,n_agents,final_pos,OdomStruct):
    """
    --------------------------------------------------------------------------
    template_optimizer: tuning parameters
    --------------------------------------------------------------------------
    """
    simulator = do_mpc.simulator.Simulator(model)
    data = do_mpc.data.MPCData(model)

    def get_Odom(model):
        from main import get_state, get_u
        # my_x = get_state(my_ID)
        my_x = OdomStruct[my_ID]
        print('my_x:')
        print(my_x.T)
        tvp_temp = simulator.get_tvp_template()
        n_horizon = 20
        for i in (j for j in range(n_agents) if j!= my_ID):
            x_var_name = 'x'
            # x_curr = get_state(i)
            x_curr = OdomStruct[i]
            u_var_name = 'u_ORCA' + str(i+1)
            n_var_name = 'n_ORCA' + str(i+1)
            sign_var_name = 's_ORCA' + str(i+1)

            delT = 0.1
            for k in range(n_horizon+1):
                # tvp_temp['_tvp',k,G_var_name]=G

                try: 
                    _X = data.prediction(('_x',x_var_name,0))
                    _Y = data.prediction(('_x',x_var_name,1))
                    _Z = data.prediction(('_x',x_var_name,2))
                    _VX = data.prediction(('_x',x_var_name,3))
                    _VY = data.prediction(('_x',x_var_name,4))
                    _VZ = data.prediction(('_x',x_var_name,5))

                    ## try changing the prediction time step
                    X = _X[0,k,0]
                    Y = _Y[0,k,0]
                    Z = _Z[0,k,0]
                    VX = _VX[0,k,0]
                    VY = _VY[0,k,0]
                    VZ = _VZ[0,k,0]
                    delT = 0.1

                    if X==0 and Y==0 and Z==0:
                        # pos = multi_agent_params['init_pos']
                        X = my_x[0] + delT*my_x[3]*k
                        Y = my_x[1] + delT*my_x[4]*k
                        Z = my_x[2] + delT*my_x[5]*k
                        VX = my_x[3]
                        VY = my_x[4]
                        VZ = my_x[5]
                        #print('inside if')
                    
                    # print(k)
                except: 
                    X = my_x[0] + delT*my_x[3]*k
                    Y = my_x[1] + delT*my_x[4]*k
                    Z = my_x[2] + delT*my_x[5]*k
                    VX = my_x[3]
                    VY = my_x[4]
                    VZ = my_x[5]

                p_a = np.array([[X[0]],[Y[0]]])
                p_b = np.array([[x_curr[0]+k*x_curr[3]*delT],[x_curr[1]+k*x_curr[4]*delT]])
                v_a = np.array([[VX[0]],[VY[0]]])
                v_b = np.array([[x_curr[3]],[x_curr[4]]])
                r_a = 1
                r_b = 1
                tau = 2
                print(v_a.shape)


                #(u,n,sign) = get_u(p_a, p_b, v_a, v_b, r_a, r_b, tau)
                sign  = 1
                u = np.array([[0.1],[0.5]])
                n = np.array([[0.1],[0.5]])

                tvp_temp['_tvp',k,u_var_name] = u
                tvp_temp['_tvp',k,n_var_name] = n
                tvp_temp['_tvp',k,sign_var_name] = sign
                tvp_temp['_tvp',k,'v_a'] = v_a
            

        return tvp_temp

    # mpc.set_tvp_fun(get_Odom(model,my_ID,n_agents,OdomStruct))
    simulator.set_tvp_fun(get_Odom)


    simulator.set_param(t_step = 0.1)

    simulator.setup()

    return simulator
