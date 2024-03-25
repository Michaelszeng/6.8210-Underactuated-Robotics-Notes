# Trajectory Optimization

The big idea: Stop searching for "policies" ($\forall x$ initial conditions); only search for control given $x_{now}$.

Breaks the curse of dimensionality.

Think about it: control policies make sense for simple systems like a pendulum; but for a humanoid, the robot will never visit all of its possible states anyway (the number of possible states scales exponetially with DoF). Control policies don't make sense.

## Problem Definition

 - $\dot{x} = f(x,u)$
 - $x_0$
 - $u(t)$ over $t \in [t_0, t_f]$
 - finite horizon cost: $J_u(x_0) = \ell_f(x(t_f)) + \int_{t_0}^{t_f} \ell(x(t),u(t))~dt$
   - $\ell_f(x(t_f))$ = termination cost

Trajectory optimization problem:

$$ \begin{align*}
    \min_u \quad & J_u(x_0) = \ell_f(x(t_f)) + \int_{t_0}^{t_f} \ell(x(t),u(t))~dt \\
    \text{subject to} \quad & \dot{x}(t) = f(x(t), u(t)) \\
    & x(t_0) = x_0
\end{align*} $$

May have other constraints (non-collision, input limits, etc.).

To make this problem solvable, $u(t)$ will have to be parameterized by a finite number of decision variables. One can already see that the problem of parameterization is much easier for traj. opt. than solving for a global control policy; the dimension of the parameterization of $u(t)$ may grow linearly with time and state space dimension, while, i.e. for Value Iteration on a mesh, the dimension of the mesh grows exponentially with dimension of the state space.


## Linear Systems, Discrete Time (Convex Optimization)

Dynamics of the form: $x[n+1] = Ax[n] + Bu[n]$.

### Direct Transcription

Goal:

$$ \begin{align*}
    \min_{x[\cdot], u[\cdot]} \quad & l_f(x[N]) + \sum_{n=0}^{N-1} l(x[n], u[n]) \\
    \text{subject to} \quad & x[n+1] = Ax[n] + Bu[n], & \forall n \in [0, N-1] \\
    & x[0] = x_0 \\
    & \text{other linear constraints...}
\end{align*} $$

Commonly, another constraint is for $x[N] = x_f$.

The idea of "Direct Transcription" is to model $x[\cdot]$ as decision variables and dynamics as a constraint.

With a linear system and a linear or quadratic cost, we could solve this using LP or QP.

In the case of a quadratic cost, you might notice this problem formulation is similar to LQR (except, of course, LQR solves a global policy). LQR is easier to solve than a QP too. The main benefit of this formulation is that you can specify other linear constraints for your trajectory; this is not possible with LQR.

We can also pick $l_f(x[N]) = x^T S x$, where $S$ is the solution to the LQR formulation of this problem. What this intuitively means is that, beyond $t=N$, we approximate the cost of the rest of time with the optimal solution to the unconstrained problem.

Also note--$N$ must be selected manually; if $N$ were a decision variable of the optimization, then the number of $x[\cdot]$ and $u[\cdot]$ decision variables would be variable. However, if, for example, you want to solve a minimum time problem to a particular $x_f$, you can solve the optimiztion multiple times until you find the minimum $N$ where a feasible answer is still produced.

#### Discretization Errors

With discrete-time dynamics, there is some discretization error. For example, if you have continuous dynamics:

$$ \dot{x} = Ax + Bu $$

This lends to discrete-time dynamics (using Euler Integration):

$$ x[n+1] = x[n] + \Delta t * (Ax[n] + Bu[n]) $$

Clearly, we are making a constant $\dot{x}$ assumption each $\Delta t$ time step, which loses accuracy.

With linear systems only, there is a possibility of removing this error by taking an actual integration of $\dot{x}$: 

$$ x[n+1] = x[n] + \int_{t_n}^{t_n + \Delta t} (Ax[n] + Bu[n])dt = \Delta t*(e^{A*dt} x[n] + Bu[n]) $$

However, with discreet time systems, there is one discrtization error that is not possible to resolve: $u$ is only parameterized at each time step (rather than being a continuous function), losing accuracy and optimality.


### Direct Shooting

In Direct Transcription, having $x[n]$ as a decision variable, and including the dynamics as a constraint, leads to both extra decision variables and constraints. Instead, we can just formulate our optimization as a single constraint, applying the dynamics in "forward simulation" for $n$ steps:

$$ \begin{align*}
x[1] &= Ax[0] + Bu[0] \\
x[2] &= A(Ax[0] + Bu[0]) + Bu[1] \\
x[n] &= A^nx[0] + \sum_{k=0}^{n-1} A^{n-1-k}Bu[k]  \end{align*}$$

Notice how this is still linear in the decision variables $u$ (can still be solved with convex optimization given a convex cost). The optimization then does not even need a constraint related to the dynamics (we plug the equation for $x[n]$ directly into the cost function):

$$ \begin{align*}
    \min_{u[\cdot]} \quad & l_f(x[N]) + \sum_{n=0}^{N-1} l(A^nx[0] + \sum_{k=0}^{n-1} A^{n-1-k}Bu[k], u[n]) \\
\end{align*} $$

Except, if you want a $x[N] = x_f$ constraint, you would need to specify $x[N]$ as $A^Nx[0] + \sum_{k=0}^{N-1} A^{N-1-k}Bu[k]$.

#### Drawbacks of Direct Shooting

Usually, numerically infeasible (requires explicitely computing $A^n$, which usually is not possible on a 64-bit machine).

Also, puts a huge weight on $u[0]$ compared to $u[N]$ ($u[0]$ is multiplied by $A$ to a much higher power), which makes the optimization numerically difficult. While this is true in real life (your first action matters more), this is better distributed in the constraints in Direct Transcription, where the solver can enforce constraints in both directions (i.e. modify an earlier action to fit withn a constraint relative to a later action).

In addition, if you do have many constraints on $x$ (i.e. $x[n] \leq 2 ~\forall ~n$), each constraint requires an instance of $A^nx[0] + \sum_{k=0}^{n-1} A^{n-1-k}Bu[k]$, so the efficiency is quickly lost. It's more natural to simply have $x[\cdot]$ as decision variables in practice.

Also, the "sparsity" of the constraints (each constraint touches a small number of decision variables) in Direct Transcription makes it not too bad to solve. 

Direct Transcription is more common in practice.


## Non-Linear Systems, Discrete Time (Non-Convex Optimization)

Direct Transcription and Direct Shooting still work, just becomes a nonconvex optimization.

General formulation of Direct Transcription:

<center><img src="Media/direct_transcription_nonlinear.png" style="width:45%"/></center><br />

General formulation of Direct Shooting is also ~identical to the linear formulation, except you compose the nonlinear dynamics.

To better approximate continuous-time dynamics, you similarly perform an integration over $\dot{x}$ instead of an Euler integration:

<center><img src="Media/direct_transcription_nonlinear_continuous_time.png" style="width:45%"/></center><br />

There are also many numerical integrators that can perform this operation with varying speed/accuracy (https://drake.mit.edu/doxygen_cxx/group__integrators.html0). 


### Direct Collocation

In general, the formulation is very similar to direct transcription, except the input trajectory and state trajectory are parameterized as piecewise polynomial functions (specifically as first-order polynomials,and cubic polynomials, respectively).

The decision variables for the optimization are simply sample points in $u(t)$ and $x(t)$; for $u(t)$ (a first-order, linear polynomial), two samples fully define the trajectory. For $x(t)$ (a cubic spline), two samples, along with two derivatives at those samples (which can be computed using system dynamics from $x(t)$ and $u(t)$), can fully define the $x(t)$ trajectory. 

Clarification: there is a separate 1st order hold for $u(t)$ and cubic polynomial for $x(t)$ between each sample point.

The optimization does require constraints on the dynamics of the system: $x(t_{k+1}) = x(t_k) + \int_{t_k}^{t_{k+1}} f(x(t), u(t)) dt$. These constraints are applied at each collocation point of the system. The collocation points are chosen at the midpoints (in the time-axis) between each breakpoint. 

<center><img src="Media/collocation.png" style="width:45%"/></center><br />

where each $t(k)$ is a breakpoint time, and $h$ is the time step. Euler integration is used to apply the dynamics from $x(t_k)$ (sample point) to $x(t_{c,k})$ (collocation point). 

Overall, the optimization is expressed like so:


$$ \begin{align*}
    \min_{x[\cdot], u[\cdot]} \quad & l_f(x[N]) + \sum_{n=0}^{N-1} \Delta t * l(x[n], u[n]) \\
    \text{subject to} \quad & \dot{x}(t_{c,n}) = f(x(t_{c,n}), u(t_{c,n})), & \forall n \in [0, N-1] \\
    & x[0] = x_0 \\
    & \text{other linear constraints...}
\end{align*} $$

(Sidenote: we're also multiplying $\Delta t$ in the summation in the cost function because this is a continuous time formulation, where $\ell()$ returns the rate of change of cost.)


## Trajectory Stabilization

Stabilization = how to follow the trajectory accurately. Generally an easier problem than solving for a globally optimal trajectory.

### Model Predictive Control (MPC)

**Repeat every time step:**
1. Estimate current state $\hat{x}$
2. Solve traj opt w/ $x[0] = \hat{x}$ for $N$ steps into the future
3. Execute $u[0]$

Note: you must solve the traj opt multiple steps into the future even if you discard $u[1] ... u[N]$, since you cannot have an "optimal control" unless you consider the future.

Recursive feasibility: if a feasible solution found in one time step, should not be lost in future time steps. (this is true if the model of the dynamics is accurate)


### Linearizing around Trajectory

Call $x_0(t)$ and $u_0(t)$ the trajectory points at time $t$. We will linearize around these points.

Then, $\tilde{x} = x - x_0(t)$ and $\tilde{u} = u - u_0(t)$.

Performing the linearization using a 1st-order Taylor Series:

$$\begin{align*}
    \dot{x} &= f(x_0(t), u_0(t)) + \frac{\delta f}{\delta x} \bigg |_{x_0(t),u_0(t)}(x-x_0) + \frac{\delta f}{\delta u}\bigg |_{x_0(t),u_0(t)} (u-u_0) \\ 
    &= \dot{x}_0(t) + \frac{\delta f}{\delta x}\bigg |_{x_0(t),u_0(t)} (x-x_0(t)) + \frac{\delta f}{\delta u}(u-u_0(t)) \\
    \dot{\tilde{x}} &= A(t)\tilde{x} + B(t)\tilde{u}

\end{align*}$$

Notice how $A$ and $B$ are no longer constant--we call this now a time-varying system.

LQR still works even if $A$ and $B$ are time-varying; so, to control this, we use finite-horizon LQR:

$$ \min_{u(t)} \int_0^{t_f} \tilde x^T(t)Q \tilde x(t) + \tilde u^T(t)R \tilde u(t) ~dt $$

$$\dot{\tilde x}(t) = A(t) \tilde x(t) + B(t) \tilde u(t)$$

Then the solution looks like this (the optimal cost to go is now a function of time):

$$J^*(x,t) = \tilde x^TS(t) \tilde x $$

$$\tilde u = -K(t) \tilde x$$