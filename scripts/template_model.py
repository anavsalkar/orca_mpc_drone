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


def template_model(my_ID, n_agents):
    """
    --------------------------------------------------------------------------
    template_model: Variables / RHS / AUX
    --------------------------------------------------------------------------
    """
    model_type = 'discrete' # either 'discrete' or 'continuous'
    model = do_mpc.model.Model(model_type)

    # Simple oscillating masses example with two masses and two inputs.
    # States are the position and velocitiy of the two masses.

    # States struct (optimization variables):
    _x = model.set_variable(var_type='_x', var_name='x', shape=(12,1))
    _va = model.set_variable(var_type='_tvp', var_name='v_a', shape=(2,1))

    # Input struct (optimization variables):
    _u = model.set_variable(var_type='_u', var_name='u', shape=(4,1))

    for i in (j for j in range(n_agents) if j!= my_ID):
        u_var_name = 'u_ORCA' + str(i+1)
        n_var_name = 'n_ORCA' + str(i+1)
        sign_var_name = 's_ORCA' + str(i+1)
        v_b_var_name = 'v_b' + str(i+1)
        _u_ORCA = model.set_variable(var_type='_tvp', var_name=u_var_name, shape=(2,1))
        _n_ORCA = model.set_variable(var_type='_tvp', var_name=n_var_name, shape=(2,1))
        _s_ORCA = model.set_variable(var_type='_tvp', var_name=sign_var_name)
        _v_b = model.set_variable(var_type='_tvp', var_name=v_b_var_name, shape=(2,1))

    # Set expression. These can be used in the cost function, as non-linear constraints
    # or just to monitor another output.
    model.set_expression(expr_name='cost', expr=sum1(_x**2))
    
    g = 9.81
    delT = 0.1
    
    A = np.array([[0,0,0,1,0,0,0,0,0,0,0,0],
                  [0,0,0,0,1,0,0,0,0,0,0,0],
                  [0,0,0,0,0,1,0,0,0,0,0,0],
                  [0,0,0,0,0,0,0,g,0,0,0,0],
                  [0,0,0,0,0,0,-g,0,0,0,0,0],
                  [0,0,0,0,0,0,0,0,0,0,0,0],
                  [0,0,0,0,0,0,0,0,0,1,0,0],
                  [0,0,0,0,0,0,0,0,0,0,1,0],
                  [0,0,0,0,0,0,0,0,0,0,0,1],
                  [0,0,0,0,0,0,0,0,0,0,0,0],
                  [0,0,0,0,0,0,0,0,0,0,0,0],
                  [0,0,0,0,0,0,0,0,0,0,0,0],])
    
    B = np.array([[0,0,0,0],
                  [0,0,0,0],
                  [0,0,0,0],
                  [0,0,0,0],
                  [0,0,0,0],
                  [1,0,0,0],
                  [0,0,0,0],
                  [0,0,0,0],
                  [0,0,0,0],
                  [0,1,0,0],
                  [0,0,1,0],
                  [0,0,0,1],])
    
    
    

    x_next = _x + delT*(A@_x+B@_u)
    model.set_rhs('x', x_next)

    model.setup()

    return model
