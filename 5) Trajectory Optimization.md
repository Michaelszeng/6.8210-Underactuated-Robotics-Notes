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

Note that Direct Transcription can be used with non-linear systems--instead of constraining $x[n+1] = Ax[n] + Bu[n]$, you have $x[n+1] = f(x[n], u[n])$.

#### Discretization Errors

With discrete-time dynamics, there is some discretization error. For example, if you have continuous dynamics:

$$ \dot{x} = Ax + Bu $$

This lends to discrete-time dynamics (using Euler Integration):

$$ x[n+1] = x[n] + \Delta t * (Ax[n] + Bu[n]) $$

Clearly, we are making a constant $\dot{x}$ assumption each $\Delta t$ time step, which loses accuracy.

With linear systems only, there is a possibility of removing this error by taking an actual integration of $\dot{x}$: 

$$ x[n+1] = x[n] + \int_{t_n}^{t_n + \Delta t} (Ax[n] + Bu[n])dt = \Delta t*(e^{A*dt} x[n] + Bu[n]) $$

However, with discreet time systems, there is one discretization error that is not possible to resolve: $u$ is only parameterized at each time step (rather than being a continuous function), losing accuracy and optimality.


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

The basic problem with direct transcription/shooting/collocation is that they solve for the trajectory given the initial state... and that is all. They don't use any feedback to ensure the robot actually follows that trajectory in real life.

Therefore, direct transcription/shooting/collocation are not enough in real life. Real life has small disturbances (plus, if you are using Euler Integration or an approximated method of integration to roll forward your dynamics, this introdues more inaccuracy) that will cause the system to miss the trajectory.

This is where Trajectory Stabilization comes into play.

### Local LQR (Linearizing around Trajectory)

Call $x_0(t)$ and $u_0(t)$ the trajectory points at time $t$. We will linearize around these points.

Then, $\tilde{x}(t) = x(t) - x_0(t)$ and $\tilde{u}(t) = u(t) - u_0(t)$.

Performing the linearization using a 1st-order Taylor Series:

$$\begin{align*}
    \dot{x}(t) &= f(x_0(t), u_0(t)) + \frac{\delta f}{\delta x} \bigg |_{x_0(t),u_0(t)}(x-x_0) + \frac{\delta f}{\delta u}\bigg |_{x_0(t),u_0(t)} (u-u_0) \\ 
    &= \dot{x}_0(t) + \frac{\delta f}{\delta x}\bigg |_{x_0(t),u_0(t)} (x-x_0(t)) + \frac{\delta f}{\delta u}(u-u_0(t)) \\
    \dot{\tilde{x}}(t) &= A(t)\tilde{x} + B(t)\tilde{u}

\end{align*}$$

Notice how $A$ and $B$ are no longer constant--we call this now a time-varying system.

LQR still works even if $A$ and $B$ are time-varying, and given a finite horizon ($t_f$ = time trajectory ends). The problem formulation looks almost identical to classic LQR:

$$ \min_{u(t)} \int_{t_{now}}^{t_f} \tilde x^T(t)Q \tilde x(t) + \tilde u^T(t)R \tilde u(t) ~dt $$

$$\dot{\tilde x}(t) = A(t) \tilde x(t) + B(t) \tilde u(t)$$

The solution looks like this (the optimal cost to go is now a function of time because the horizon is finite (intuitively, being at a far-away state at $t=0$ is much less bad thn being at a far-away state at $t$ close to $t_f$))  (where $S(t) \succ 0$):

$$J^*(x,t) = \tilde x^TS(t) \tilde x $$

$$\tilde u^* = -K(t) \tilde x$$

This simple control strategy--re-linearizing and re-applying LQR with a finite-horizon at a fixed loop speed--can achieve very robust control with real world disturbances.

Note: LQR assumes the target state can be reached by $t_f$. In the classic (infinite-horizon) LQR case, the target state can obviously be reached in infinite time. In the finite-horizon LQR case, we are giving the controller authority over the duration of the trajectory.

Note: using LQR to solve means we cannot add other constraints like input limits or state constraints.

Note: you ***can*** have even $Q$ and $R$ be functions of time.

Note: Having a cost-to-go function that is a function of time --> HJB equation is slightly different (has additional partial derivative w.r.t time term): 
$$ 0 = \min_u \bigg [\ell(x, u) + \frac{\delta J^*}{\delta  x} \bigg|_{x,t} f_c(x, u) + \frac{\delta J^*}{\delta  t} \bigg|_{x,t} \bigg ] $$

This means the solution for $S(t)$ is also different: $Q-S(t)B(t)R^{-1}B^T(t)S(t) + 2S(t)A(t) = -\dot{S}(t)$




#### Limitation of Time-Varying Linearization

Trajectories are solved as functions of time. Therefore, if there are unexpected forces/dynamics applied on the system, the nominal point around which linearization is done, $x_0, u_0$, will continue moving forward in time even though the system is no longer following the trajectory; in this case, $\tilde{x}$ and $\tilde{u}$ can increase significantly, making the linearization less and less accurate. The control policy can fail in this case.

Simply remapping time by always performing the linearization around the nearest $x, u$ on the trajectory to the current $x, u$, can introduce other instabilities, so is not a good option either. TODO: give example


### Time-Varying Lyapunov

If we locally linearize as with the "Local LQR" method, we can analyze the stability of the resulting time-varying system. We can write time-varying Lyapunov conditions:

$$\forall t \quad V(x, t) \succ 0, ~V(0, t) = 0$$
$$ \dot{V}(t, x) = \frac{\delta V}{\delta x} f(x) + \frac{\delta V}{\delta t} \preceq 0$$

Or, if we're intersted in how much perturbation around the planned trajectory the system can take to still arrive at the target state (i.e. certifying regions of attraction):

$$ V(x, t) \leq \rho(t) \implies \dot{V}(x, t) \leq \dot{\rho}(t) \quad \forall t \in [t_0, t_f] $$

where we can parameterize $\rho(t)$ as a polynomial. 

Using LQR as our controller, we get an optimal cost-to-go function $ J^*(x) = x^T Sx $ (where $S \succ 0$) that we can use as our Lyapunov function (recall that cost-to-go functions are weakly decreasing). Then, $\rho(t)$ can be solved with an SOS optimization:

Common in practice is to actually sample discrete times along the trajectory and solve for $\rho$ at each sample to get an approximation of $\rho(t)$. To solve for each $\rho$, we use the method discussed in *4) Lyapunov Analysis*:

<center><img src="Media/roa_opt.png" style="width:45%"/></center><br />

where $d$ is a fixed positive integer, and we use the appropriate $A(t), B(t), K(t), S(t)$ for the sampled time.

Knowing $\rho(t)$ gives us an idea of the size of the region of stability for the system and controller (this is simply an analysis tool).


### Model Predictive Control (MPC)

**Repeat every time step:**
1. Estimate current state $\hat{x}$
2. Solve traj opt w/ $x[0] = \hat{x}$ for $N$ steps into the future ("receding horizon"), i.e. using direct collocation
3. Execute $u[0]$ and let dynamics evolve

Note: you must solve the traj opt multiple steps into the future even if you discard $u[1] ... u[N]$, since you cannot have an "optimal control" unless you consider the future.

Recursive feasibility is an important notion: if a feasible solution is found in one time step, it should not be lost in future time steps. The basic formultion of MPC does not have this; by having a receding horizon, essentially every time step, MPC adds a new constraint at a future time that has not been considered before; this constraint could cause sudden infeasibility. There are a few ideas to combat this:
 - Add a constraint that $x[k+N+1] = x^*$ (where $k$ is the current time step), where $x^*$ is the target state at the end of the trajectory. If the controller can get to $x^*$ by $t = k + N$, then it is sure to be feasible to get there in time steps beyond that. (This only works for short trajectories or very long horizons)
    - Extension: if you can define a "safe set" where you know the system can converge to $x^*$ from this set, constraint $x[k+N+1]$ to be in this set.
 - "heuristic penalty / heuristic constraint on the last couple of states" - TODO: update this


### Linear Model Predictive Control (MPC)

Same principle as MPC, but perform a linearization of the system around the current state at each time step, then perform linear optimal control (general MPC might just do a nonlinear optimization), i.e. with direct transcription or direct shooting. The reason for this is that convex optimization can give you guarantees of optimality and feasibility that non-convex optimization cannot.

This is also very similar to Local LQR, except we don't restrict ourselves to an LQR cost and optimization, so Linear MPC can still solve optimizations with other linear constraints. 

The downside to Linear MPC compared to Local LQR is 

You can, in fact combine Local LQR and Linear MPC, using LQR if you know you are far from the linear constraints, and switching to MPC otherwise. Local LQR is more computationally efficient, and Local LQR is more tractable for analysis (i.e. SOS optimization for verifying regions of attraction).



## Case Study: Perching Plane
1. Direct Collocation to solve trajectory
2. Linearization + LQR to stabilize along trajectory
3. Cost-to-go from LQR as Lyapunov function (cost-to-go strictly decreases each time step)
4. Find largest $\rho(t)$


## iLQR (Iterative LQR)

Summary: Trajectory Optimization using LQR stabilization.

Similar to Finite Horizon LQR, iLQR begins with a nominal trajectory (solved i.e. with direct collocation) $x[\cdot], u[\cdot]$, then performs a linearization of state dynamics around a nominal point $x_0, u_0$. iLQR also performs a quadraticization of the cost function (if it were not already quadratic).

Each iteration of the algorithm, there are two steps:
1. Backward Pass: Same as Finite Horizon LQR, LQR is solved for the optimal control policy and cost-to-go at the current time given the current optimal trajectory
2. Forward Pass: Applies the optimal cost-to-go to the system dynamics 