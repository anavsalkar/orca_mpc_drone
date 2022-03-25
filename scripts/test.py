import numpy as np
from numpy.linalg import norm
from math import pi, cos, atan2, asin, tan
import matplotlib.pyplot as plt
from main import get_u


# def get_u(p_a, p_b, v_a, v_b, r_a, r_b, tau):
#     # print('0')
#     m = (p_b-p_a)
#     m = m/norm(m)
#     # print('1')
#     v_rel = v_a - v_b
#     theta_m = atan2(m[1],m[0])
#     print(theta_m)
#     print(norm(r_b+r_a)/norm(p_b-p_a))
#     theta_i = asin(norm(r_b+r_a)/norm(p_b-p_a))
#     plus_line_m = tan(theta_m+theta_i)
#     minus_line_m = tan(theta_m-theta_i)
#     lb = cos(pi/2 + theta_i)
#     v_rel_p = (v_a-v_b)-(p_b-p_a)/tau
#     if np.dot(m.T,v_rel_p/norm(v_rel_p))[0][0] < lb:
#         n = (-(v_a-v_b)+(p_b-p_a)/tau)
#         mag = norm(n)-((r_a+r_b)/tau)
#         if mag < 0:
#             sign = -1
#         else: 
#             sign = 1
#         n = n/norm(n)
#         # print((r_a+r_b)/tau)
#         # print(n)
#         # print(mag)
#         u = mag*n
#         print('circle!')

#     else:
#         u1_plus = np.array([[1],[plus_line_m]])
#         u1_minus = np.array([[1],[minus_line_m]])
#         # print(v_rel.shape)
#         # print(u1_plus.shape)
#         p_hat_plus = (np.dot(v_rel.T,u1_plus)/np.dot(u1_plus.T,u1_plus))*u1_plus
#         p_hat_minus = (np.dot(v_rel.T,u1_minus)/np.dot(u1_minus.T,u1_minus))*u1_minus

#         u_plus = p_hat_plus - v_rel
#         u_minus = p_hat_minus - v_rel

#         if np.dot(m.T,v_rel/norm(v_rel))[0][0] > cos(theta_i):
#             sign = -1
#         else:
#             sign = 1

#         if norm(u_plus) < norm(u_minus):
#             u = u_plus
#             print('plus')
#         else:
#             u = u_minus
#             print('minus')

#     n = u/norm(u)

#     return (u, n, sign)
    ## sign -> -1 for inside, 1 for outside (-1,1)

p_a = np.array([[6],[6]])
p_b = np.array([[-1],[-1]])
v_a = np.array([[-13],[-20]])
v_b = np.array([[0],[0]])
r_b = 1
r_a = 1
tau = 1

# p_a = np.array([[0.1703],[0.1932]])
# p_b = np.array([[0.38],[-0.001]])
# v_a = np.array([[-0.0012],[0.1]])
# v_b = np.array([[0.09],[0]])
# r_b = 0.1
# r_a = 0.1
# tau = 1
print(p_a.shape)
(u,n,sign) = get_u(p_a, p_b, v_a, v_b, r_a, r_b, tau)
print(u.T)
print(n.T)
print(((v_a-v_b)+u).T)

figure, axes = plt.subplots()
Drawing_uncolored_circle = plt.Circle( ((p_b-p_a)[0]/tau, (p_b-p_a)[1]/tau ),
                                      ((r_a+r_b)/tau) ,
                                      fill = False )

# Drawing_uncolored_circle = plt.Circle(( 12, 15 ),
#                                       ((r_a+r_b)/2) ,
#                                       fill = True )

# plt.quiver((v_a-v_b).T, ((v_a-v_b)+5*n).T)
# plt.quiver((v_a-v_b)[0], (v_a-v_b)[1] , u[0], u[1], scale = 1)
axes.arrow(float((v_a-v_b)[0]), float((v_a-v_b)[1]), float(u[0]), float(u[1]))
axes.arrow(0, 0, float(((v_a-v_b)+u)[0]), float(((v_a-v_b)+u)[1]))

axes.set_aspect( 1 )
axes.add_artist( Drawing_uncolored_circle )
plt.xlim(-25,2)
plt.ylim(-25,2)
# plt.xlim(min((p_b-p_a)[0]/tau,(v_a-v_b)[0])-5,max((p_b-p_a)[0]/tau,(v_a-v_b)[0])+5)
# plt.ylim(min((p_b-p_a)[1]/tau,(v_a-v_b)[1])-5,max((p_b-p_a)[1]/tau,(v_a-v_b)[1])+5)
plt.title( 'Circle' )
plt.show()


