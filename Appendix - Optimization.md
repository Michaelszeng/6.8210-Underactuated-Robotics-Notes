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

It's clear how a convex cost function is necessary. In addition, linear constraints don't cause a problem to lose its convexity, since linear constraints essentially draw hyperplanes in the decision-variable space which construct a convex set.

### Making Absolute Value Objective solveable with Linear Programming

Instead of solving: 
$$\min_{x,u} \sum_n^{N-1} |x| + |u| $$

Insert "slack variables" and solve:
$$\min_{x, u, s_x, s_u} \sum_n^{N-1} s_x[n] + s_u[n] ~~~~s.t.~~~s_x[n] \geq x[n], ~s_x[n] \geq -x[n], ~s_u[n] \geq u[n], ~s_u[n] \geq -u[n]$$

### Exploiting Sparsity

When constraints touch only a small number of decision variablese each, this can be exploited by some solvers for faster solves.

### SNOPT 

Sequential Quadratic Programming (SQP): Repeatedly takes local quadratic approximations of optimization landscape (by taking gradients at multiple points), linear approximations of the constraints, then solves QP, the repeats. Similar to gradient descent (but is 2nd order) (so it converges faster and gracefully handles constraints).

### SDP Feasibility Problems (and what to set the Cost as)

Often, the goal of SDP problems is to find any feasible solution. Even so, it is best to add some cost function to give a more numerically-controlled answer.

A frequent choice is $ \min tr(P) $. Why:
 - $tr(P) = $ sum of eigenvalues of $P$. The definition of a PSD matrix is non-negative eigenvalues--so we're asking for the matrix to be PSD but not to be too extreme.
 - $P$ won't just go to 0 in practice since we actually write the PSD constraint to be something like $P - \epsilon \succeq 0; so $P$ will be at least $\epsilon$
 - Minimizing trace can also produce sparser matrices.

### Non-collision Constraints

Strategy 1: binary result from collision engine (NOT GOOD; provides no gradient information for solver).

Strategy 2: signed distance funtions (SDF) between all collision pairs: $\phi_{i,j}(q)$. Add a single constraint on the minimum signed distance between any collision pair: 
$$\min_{i,j} \phi_{i.j}(q) \geq d_{min}$$

In addition, you would typically use a $\text{softmin}$ to make the function differentiable.

Note: in practice, collision engine will do a rough approximation (i.e. axis-aligned bounding boxes) of SDF to prune away irrelevant collision pairs, then only compute an accurate signed distance for close objects.

Note: this is always a non-convex constraint (think about it--you are bounding all valid states to be outside of an object--then you can easily pick two states that fail the convexity test (the line between them stays in the set)).