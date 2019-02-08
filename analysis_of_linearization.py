import numpy as np
import control

g = 9.8
L = 1.5
m = 1.0
M = 5.0
d1 = 1.0
d2 = 0.5

# Pendulum up
# Eigen val of A : array([[ 1.        , -0.70710678, -0.07641631,  0.09212131] )
A = np.array([\
            [0,1,0,0], \
            [0,-d1, -g*m/M,0],\
            [0,0,0,1],\
            [0,0,(M+m)*g/(M*L),-d2] ] )
B = np.array( [ [0, 0 ],[1./M, g*m*np.pi/(2*M) ], [0,0], [-1./(M*L), -g*(m+M)*np.pi / (2*M*L) ]  ])


# Pendulum Down - Verified correct.
# Eigen Values of this: array([ 0.00+0.j        , -1.00+0.j        , -0.25+2.78881695j,       -0.25-2.78881695j])
# A = np.array([\
#             [0,1,0,0], \
#             [0,-d1, -g*m/(2*m+M),0],\
#             [0,0,0,1],\
#             [0,0,-(M+m)*g/(M*L),-d2] ] )

print np.linalg.matrix_rank( control.ctrb( A, B ) )
print np.linalg.eig( A )
