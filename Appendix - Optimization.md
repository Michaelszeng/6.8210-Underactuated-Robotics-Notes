# Problem Formulations

 - Linear Program (LP): linear cost ($c^Tx$), linear constraints ($Ax \leq b$).
 - Quadratic Program (QP): quadratic cost ($\frac{1}{2} x^TQx + c^Tx$), linear constraints ($Ax \leq b$).
 - 2nd Order Cone Program (SOCP): quadratic cost ($\frac{1}{2} x^TQx + c^Tx$), conical constraints (?)
 - Semi-deifinite Program (SDP): linear cost ($c^Tx$), linear constraints ($Ax \leq b$) + P.S.D matrix constraints (i.e. contraining a matrix to be PSD).
 - Sum of Squares (SOS): more of a "frontend" for SDP; allows you to pass in SOS constrants ("$p_\alpha(x)$ is SOS" where $p_\alpha(x)$ is a polynomial w/coefficients $\alpha$).
   - will automatically figure out the right nonlinear basis functions for $\phi^T(x) P \phi(x)$ and solve for $\alpha$ and $P$ using SDP.

<br />
<br />

# Optimization Tricks

A core intuition: adding constraint can only increase the total cost.

### Solve Time

For simple conex optimizations (QP, LP), adding decision variables is cheap. In general, adding decision variables is cheaper than adding constraints.

### Intuition of Convex Optimization

It's clear how a convex cost function is necessary. In addiiton, linear constraints don't cause a problem to lose its convexity, since linear constraints essentially draw hyperplanes in the decision-variable space which construct a convex set.

### Making Absolute Value Objective solveable with Linear Programming

Instead of solving: 
$$\min_{x,u} \sum_n^{N-1} |x| + |u| $$

Insert "slack variables" and solve:
$$\min_{x, u, s_x, s_u} \sum_n^{N-1} s_x[n] + s_u[n] ~~~~s.t.~~~s_x[n] \geq x[n], ~s_x[n] \geq -x[n], ~s_u[n] \geq u[n], ~s_u[n] \geq -u[n]$$

### Exploiting Sparsity

When constraints touch only a small number of decision variablese each, this can be exploited by some solvers for faster solves.

### SNOPT 

Sequential Quadratic Programming (SQP): Repeatedly takes local quadratic approximations of optimization landscape (by taking gradients at multiple points), linear approximations of the constraints, then solves QP, the repeats. Similar to gradient descent (but is 2nd order) (so it converges faster and gracefully handles constraints).