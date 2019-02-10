import numpy as np
import cv2

from InvertedPendulum import InvertedPendulum

from scipy.integrate import solve_ivp
import control
import code


class MyLinearizedSystem:
    def __init__(self):
        g = 9.8
        L = 1.5
        m = 1.0
        M = 5.0
        d1 = 1.0
        d2 = 0.5

        # Pendulum up (linearized eq)
        # Eigen val of A : array([[ 1.        , -0.70710678, -0.07641631,  0.09212131] )
        _q = (m+M) * g / (M*L)
        self.A = np.array([\
                    [0,1,0,0], \
                    [0,-d1, -g*m/M,0],\
                    [0,0,0,1.],\
                    [0,d1/L,_q,-d2] ] )

        self.B = np.expand_dims( np.array( [0, 1.0/M, 0., -1/(M*L)] ) , 1 ) # 4x1

    def compute_K(self, desired_eigs = [-0.1, -0.2, -0.3, -0.4] ):
        print '[compute_K] desired_eigs=', desired_eigs
        self.K = control.place( self.A, self.B,  desired_eigs )

    def get_K(self):
        return self.K


# Global Variables
ss = MyLinearizedSystem()

# Arbitrarily set Eigen Values
#ss.compute_K(desired_eigs = np.array([-.1, -.2, -.3, -.4])*3. ) # Arbitarily set desired eigen values

# Eigen Values set by LQR
Q = np.diag( [1,1,1,1.] )
R = np.diag( [1.] )
# K : State feedback for stavility
# S : Solution to Riccati Equation
# E : Eigen values of the closed loop system
K, S, E = control.lqr( ss.A, ss.B, Q, R )
ss.compute_K(desired_eigs = E ) # Arbitarily set desired eigen values



# This will be our LQR Controller.
# LQRs are more theoritically grounded, they are a class of optimal control algorithms.
# The control law is u = KY. K is the unknown which is computed as a solution to minimization problem.
def u( t , y ):
    u_ = -np.matmul( ss.K , y - np.array([0,0,np.pi/2.,0]) ) # This was important
    print 'u()', 't=',t, 'u_=', u_
    # code.interact(local=dict(globals(), **locals()))
    # return 0.1
    return u_[0]

# Pendulum and cart system (non-linear eq). The motors on the cart turned at fixed time. In other words
# The motors are actuated to deliver forward x force from t=t1 to t=t2.
# Y : [ x, x_dot, theta, theta_dot]
# Return \dot(Y)
def y_dot( t, y ):
    g = 9.8 # Gravitational Acceleration
    L = 1.5 # Length of pendulum

    m = 1.0 #mass of bob (kg)
    M = 5.0  # mass of cart (kg)

    d1 = 1.0
    d2 = 0.5


    x_ddot = u(t, y) - m*L*y[3]*y[3] * np.cos( y[2] ) + m*g*np.cos(y[2]) *  np.sin(y[2])
    x_ddot = x_ddot / ( M+m-m* np.sin(y[2])* np.sin(y[2]) )

    theta_ddot = -g/L * np.cos( y[2] ) -  np.sin( y[2] ) / L * x_ddot


    damping_x =  - d1*y[1]
    damping_theta =  - d2*y[3]

    return [ y[1], x_ddot + damping_x, y[3], theta_ddot + damping_theta ]

if __name__=="__1main__":

    # Verifying the correctness of linearization by evaluating y_dot
    # in two ways a) non-linear b) linear approximation

    at_y = np.array( [0,0,np.pi/2+0.1,0.002] )
    print 'non-linear', y_dot( 1.0, at_y )
    print 'linearized ', np.matmul( ss.A, at_y - np.array([0,0,np.pi/2.,0]) )  + ss.B.T * 0.1
    code.interact(local=dict(globals(), **locals()))

    # See also analysis_of_linearization to know more.



# Both cart and the pendulum can move.
if __name__=="__main__":
    # We need to write the Euler-lagrange equations for the both the
    # systems (bob and cart). The equations are complicated expressions. Although
    # it is possible to derive with hand. The entire notes are in media folder or the
    # blog post for this entry. Otherwse in essense it is very similar to free_fall_pendulum.py
    # For more comments see free_fall_pendulum.py
    sol = solve_ivp(y_dot, [0, 20], [ 0.0, 0., np.pi/2 + 0.01, 0. ],   t_eval=np.linspace( 0, 20, 100)  )


    syst = InvertedPendulum()

    for i, t in enumerate(sol.t):
        rendered = syst.step( [sol.y[0,i], sol.y[1,i], sol.y[2,i], sol.y[3,i] ], t )
        cv2.imshow( 'im', rendered )
        cv2.moveWindow( 'im', 100, 100 )

        if cv2.waitKey(0) == ord('q'):
            break
