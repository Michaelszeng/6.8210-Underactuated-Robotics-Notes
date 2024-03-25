# Linearization (of ODEs)

## Method 1: Taylor Expansion

Taylor Expansion:

<center><img src="Media/taylor_expansion.png" style="width:65%"/></center><br />

Procedure: 
1. Substitute $x = x^* + \tilde{x}$
2. For each nonlinear term, apply Taylor Series

Example:
<center><img src="Media/linearization_example.png" style="width:50%"/></center><br />

Taylor Expansions also apply for multi-variate and multi-dimensional systems:

<center><img src="Media/taylor_expansion_multivar.png" style="width:45%"/></center><br />

Example:
<center><img src="Media/linearization_example_multivar.png" style="width:55%"/></center><br />

Note: Notice that the affine component, $f(x^*)$, canceled out to 0 in both examples. This will be true when linearizing any differential equation around any $x^*$ that is a fixed point; this is because, by definition of a fixed point, $f(x^*) = \dot{x} = 0$. 

However, if we want to linearize around a non-fixed point $\bar{x}$, this affine component will persist (which makes it hard to apply linear control techniques like LQR), unless we employ a trick. By substituting $x = x^* + \tilde{x}$ we express the approximation in error coordinates $\tilde{x}$. This will also get rid of the affine component, because $f(x^*) = 0$ in the reference frame of $x^*$. For more detail, see section "Linearizing around Trajectory" in *5) Trajectory Optimization*.

## Method 2: Jacobian Matrices

Compute the Jacobian matrix:

<center><img src="Media/linearization_jacobian.png" style="width:55%"/></center><br />

Then the resulting linear (affine) system is $\dot{x} = Ax + f(x^*)$

Example:
<center><img src="Media/linearization_jacobian_example.png" style="width:60%"/></center><br />

Intuition: Just by observation, the Jacobian matrix is clearly doing the same thing as a multi-variate Taylor Expansion. All a Jacobian is is the matrix full of partial derivatives of a function; Taylor Expansion simply evaluates partial derivatives with some added/multiplied constants/variables.

Same as in Method 1 (read above for more details), we can observe that the affine component $f(x^*) = 0$ when $x^*$ is a fixed point, and when $x^*$ is not a fixed point, we can remove the affine component by shifting into error coordinates.


# Dimensionality of HJB

$$ 0 = \min_u \bigg [\ell(x, u) + \frac{\delta J^*}{\delta  x} \bigg|_x f_c(x, u) \bigg ] $$

 - $\frac{\delta J^*}{\delta  x} $ is $1 \times n$; each element is a partial derivative with respect to a different element of $x$
 - $f(x,u)$ is $n \times 1$

When manually solving for $u^*(x)$ (given a cost-to-go function), you can remove all terms from the right-hand-side that do not have $u$ terms.