import numpy as np
import cv2

from InvertedPendulum import InvertedPendulum

from scipy.integrate import solve_ivp

# Pendulum. Cart is fixed and cannot move.
# Y : [ theta, theta_dot ]
# returns expression for Y_dot.
def func( t, y ):
    g = 9.8 # Gravitational Acceleration
    L = 1.5 # Length of pendulum

    damping =  - 0.5*y[1]
    return [ y[1], -g/L * np.cos( y[0] )  + damping ]


# Only the pendulum moves the cart is stationary
if __name__=="__main__":
    # Solve ODE: theta_dot_dot = -g / L * cos( theta ) + delta * theta_dot
    #       Use state y as `[ theta ; theta_dot ]`. Then the above equation can
    #       be written as a function of y and t. `y_dot = f( t, y )`.
    #
    #  Given an inital state `y0` and limits of integration, we know the trajectory
    #       followed by the pendulum. The solution of this ODE is plotted with
    #       as a simulation.
    sol = solve_ivp(func, [0, 20], [ np.pi/2 + 0.1, 0 ],   t_eval=np.linspace( 0, 20, 300)  )


    syst = InvertedPendulum()

    for i, t in enumerate(sol.t):
        rendered = syst.step( [0,1, sol.y[0,i], sol.y[1,i] ], t )
        cv2.imshow( 'im', rendered )

        if cv2.waitKey(30) == ord('q'):
            break
