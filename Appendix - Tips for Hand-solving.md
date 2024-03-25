# Linearization

## Method 1: Taylor Expansion

Taylor Expansion:

<center><img src="Media/taylor_expansion.png" style="width:65%"/></center><br />

Procedure: 
1. Set $x = x^* + \tilde{x}$
2. For each nonlinaer term, apply Taylor Series

Example:
<center><img src="Media/linearization_example.png" style="width:50%"/></center><br />

Taylor Expansions also apply for multi-variate and multi-dimensional systems:

<center><img src="Media/taylor_expansion_multivar.png" style="width:45%"/></center><br />

Example:
<center><img src="Media/linearization_example_multivar.png" style="width:55%"/></center><br />


## Method 2: Jacobian Matrices

Compute the Jacobian matrix:

<center><img src="Media/linearization_jacobian.png" style="width:50%"/></center><br />

Then the resulting linear (affine) system is $\dot{x} = A(x-\begin{bmatrix}
a \\
b
\end{bmatrix})$

Example:
<center><img src="Media/linearization_jacobian_example.png" style="width:60%"/></center><br />

