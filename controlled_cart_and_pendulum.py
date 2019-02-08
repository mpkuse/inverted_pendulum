import numpy as np
import cv2

from InvertedPendulum import InvertedPendulum

from scipy.integrate import solve_ivp
import scipy.linalg
import code

def lqr(A,B,Q,R):
    """Solve the continuous time lqr controller.

    dx/dt = A x + B u

    cost = integral x.T*Q*x + u.T*R*u
    Implementation borrowed from : http://www.kostasalexis.com/lqr-control.html
    """
    #ref Bertsekas, p.151

    #first, try to solve the ricatti equation
    X = np.matrix(scipy.linalg.solve_continuous_are(A, B, Q, R))

    #compute the LQR gain
    K = np.matrix(np.linalg.inv(R)*(B.T*X))
    # K = np.matrix( B.T*X )

    eigVals, eigVecs = scipy.linalg.eig(A-B*K)

    return K, X, eigVals

def dlqr(A,B,Q,R):
    """Solve the discrete time lqr controller.


    x[k+1] = A x[k] + B u[k]

    cost = sum x[k].T*Q*x[k] + u[k].T*R*u[k]
    """
    #ref Bertsekas, p.151

    #first, try to solve the ricatti equation
    X = np.matrix(scipy.linalg.solve_discrete_are(A, B, Q, R))

    #compute the LQR gain
    K = np.matrix(scipy.linalg.inv(B.T*X*B+R)*(B.T*X*A))

    eigVals, eigVecs = scipy.linalg.eig(A-B*K)

    return K, X, eigVals

# if __name__=="__1main__":
#     print "Test lqr"
#     g = 9.8
#     L = 1.5
#     m = 1.0
#     M = 5.0
#     A = np.array([\
#                 [0,1,0,0], \
#                 [0,-1./M,-g*m/M,0],\
#                 [0,0,0,1],\
#                 [0,-1./(M*L),(M+m)*g/(M*L),0] ] )
#     B = np.array( [ [0],[1./M],[0],[1./(M*L)]  ])
#
#     _a, _b, _c = lqr( A, B, np.eye(4), 1. )
#     code.interact( local=locals() )



# This will be our LQR Controller. Note that this is not a PID Controller.
# LQRs are more theoritically grounded, they are a class of optimal control algorithms.
# The control law is u = KY. K is the unknown which is computed as a solution to minimization problem.
def u( t , y ):
    g = 9.8
    L = 1.5
    m = 1.0
    M = 5.0
    A = np.array([\
                [0, 1, 0                      ,0], \
                [0, -1      , -g*m/M       ,0],\
                [0, 0          , 0            ,1],\
                [0, 0, (M+m)*g/(M*L)   ,-0.5] ] )

    B = np.array( [ [0,0],[1./M,g*m*np.pi/(2.*M)],[0,0],[-1./(M*L), -g*(m+M)*np.pi/(2*M*L)]  ])

    K, _b, _c = dlqr( A, B, 1.9*np.eye(4), 1.9*np.eye(2) )
    u = np.matmul( np.array( K ), y )
    print 'u=', u, 1000*u[0]/u[1]
    # code.interact( local=locals() )
    # return -5000.*u[0]/u[1]
    return u

# Pendulum and cart system. The motors on the cart turned at fixed time. In other words
# The motors are actuated to deliver forward x force from t=t1 to t=t2.
# Y : [ x, x_dot, theta, theta_dot]
# Return \dot(Y)
def func3( t, y ):
    g = 9.8 # Gravitational Acceleration
    L = 1.5 # Length of pendulum

    m = 1.0 #mass of bob (kg)
    M = 5.0  # mass of cart (kg)


    x_ddot = u(t, y) - m*L*y[3]*y[3] * np.cos( y[2] ) + m*g*np.cos(y[2]) *  np.sin(y[2])
    x_ddot = x_ddot / ( M+m-m* np.sin(y[2])* np.sin(y[2]) )

    theta_ddot = -g/L * np.cos( y[2] ) - 1./L * np.sin( y[2] ) * x_ddot

    damping_x =  - 1.0*y[1]
    damping_theta =  - 0.5*y[3]

    return [ y[1], x_ddot + damping_x, y[3], theta_ddot + damping_theta ]



# Both cart and the pendulum can move.
if __name__=="__main__":
    # We need to write the Euler-lagrange equations for the both the
    # systems (bob and cart). The equations are complicated expressions. Although
    # it is possible to derive with hand. The entire notes are in media folder or the
    # blog post for this entry. Otherwse in essense it is very similar to free_fall_pendulum.py
    # For more comments see free_fall_pendulum.py
    sol = solve_ivp(func3, [0, 20], [ 0.0, 0., np.pi/2 + 0.01, 0. ],   t_eval=np.linspace( 0, 20, 300)  )


    syst = InvertedPendulum()

    for i, t in enumerate(sol.t):
        rendered = syst.step( [sol.y[0,i], sol.y[1,i], sol.y[2,i], sol.y[3,i] ], t )
        cv2.imshow( 'im', rendered )
        cv2.moveWindow( 'im', 100, 100 )

        if cv2.waitKey(30) == ord('q'):
            break
