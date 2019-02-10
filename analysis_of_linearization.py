import numpy as np
import control

g = 9.8
L = 1.5
m = 1.0
M = 5.0
d1 = 1.0
d2 = 0.5

# Pendulum up (linearized eq)
# Eigen val of A : array([[ 1.        , -0.70710678, -0.07641631,  0.09212131] )
_q = (m+M) * g / (M*L)
A = np.array([\
            [0,1,0,0], \
            [0,-d1, -g*m/M,0],\
            [0,0,0,1.],\
            [0,d1/L,_q,-d2] ] )

B = np.expand_dims( np.array( [0, 1.0/M, 0., -1/(M*L)] ), 1 )



# Pendulum Down - Verified correct.
# Eigen Values of this: array([ 0.00+0.j        , -1.00+0.j        , -0.25+2.78881695j,       -0.25-2.78881695j])
# A = np.array([\
#             [0,1,0,0], \
#             [0,-d1, -g*m/(2*m+M),0],\
#             [0,0,0,1],\
#             [0,0,-(M+m)*g/(M*L),-d2] ] )

#B = np.array( [] )

print 'A\n', A
print 'B\n', B

# Controllability
print '---Controllability'
print 'rank of ctrb(A,b)' , np.linalg.matrix_rank( control.ctrb( A, B ) )
print 'Eigenvalues of A ', np.linalg.eig( A )


# Pole Placement
K = control.place( A, B, [-1, -2, -4, -5] )
print '---Pole Placement\nK=\n', K

# Verification of Eigen values of A-BK
print '---Verification of Eigen values of A-BK'
print 'Eigenvalues of A-BK', np.linalg.eig( A-np.matmul(B,K) )
