# opt_m
Matlab-based, general purpose optimal control solver

Methods for trapezoidal and hermite-simpson collocation

Credits and References:

[1] Kelly, Matthew 'An Introduction to Trajectory Optimization: How to Do Your
Own Direct Collocation' 2017

[2] Betts, John T. 'Practical Methods for Optimal Control using Nonlinear 
Programming' 2001

Big thanks to Mike Sparapany for the idea for this """weekend""" project
and all his help in learning about optimal control.



Derivation of trapezoidal integration coefficients:

First equation is from Ref 1
![Alt text](pics/trap_coeff.png?raw=true)

Derivation of nonlinear equality constraint on dynamics:

All but last equation is from Ref 1
![Alt text](pics/dyn.png?raw=true)