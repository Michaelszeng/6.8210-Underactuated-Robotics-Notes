## Underactuated Systems

Dynamics of robot expressed using diff eq, relating state & control input to derivatives of state. In particular, we use 2nd-order diff eq, because we typically care about the resulting acceleration from a state & control input (why acceleration? well, $q$ and $\dot{q}$ are easy to measure with sensors; if you wanted a diff eq with jerk, you'd need a measure of $\ddot{q}$, which are usually noisy, and this is usually not necessary anyway).

General 2nd-order Dynamical System Control Equation:

$$ \ddot{q} = f(q, \dot{q}, u, t) $$

$q$ = state vector, $u$ = control command. For a fully actuated dsystem, $f$ is surjective; for each $\ddot{q}$ there exists a $u$. This is not true for underactuated systems.

Actually, dynamics of many robots is linear in torque with an added constant, so we re-express the dynamics:

$$ \ddot{q} = f_1(q, \dot{q}, t) + f_2(q, \dot{q}, t)u $$

For a robot manipulator (with rigid linkages and joints), recall the manipulator equation:

$$ M(q) \ddot{q} + C(q, \dot{q})\dot{q} = \tau_g(q) + Bu $$

where $M$ is the inertia matrix (symmetric, positive definite, invertible), and $B$ maps $u$ into generalized forces. We can re-express this in the form of the linear dynamics equation ($M^{-1}B$ represents $f_2$ and $M^{-1}[\tau_g(q) - C(q,\dot{q})\dot{q}]$ represents $f_1$):

$$ \ddot{q} = M^{-1}(q)[\tau_g(q) + Bu - C(q,\dot{q})\dot{q}] $$

As long as $B$ is full-row-rank (i.e. has rank equal to the number of dimensions of $q$, and therefore has a right inverse), $\exist u$ such that $\ddot{q}$ can achieve any value in full-dimensional space.

Sidenote 1: The manipulator equation ignores friction, but you could add a $\tau_f(q)$ to the manipulator equation to model this  (or, if you have torque-dependent friction, then $\tau_g(q)u$)

Sidenote 2: $B$ must be dimension $dim(q) \times dim(u)$, and the only way for it to be full-row-rank is if $dim(u) \geq dim(q)$. $dim(u)$ could even be much larger than $dim(q)$ if you have multiple actuators per joint; in this case the inverse of $B$ may not be unique.

If, for example, the manipulator has a free-spinning joint, then $B$ would lose a dimension, and then $\ddot{q}$ would be limited within the subspace that $B$ projects to.


### Feedback Equivalence

For full-actuated systems, control is easy. For any desired $\ddot{q}^d$:

$$ u = B^{-1} [ M(q) \ddot{q}^d - \tau_g(q) + C(q,\dot{q})\dot{q}] $$

$u$ is calculated analytically.

This is also called a "double integrator" controller. Disadvantage: requires may actuators + energy.

Also, feedback equivalence may only be possible sometimes. $B^{-1}$ could get large $\rightarrow$ demand unreasonably high torques.


### Input and State Constraints

Limits to $u$ can cause underactuation. $q$ limits (i.e. obstacles or joint limits) don't necessarily cause underactuation; they actually reduce the state space, which could make an underactuated system temporarily fully-actuated.


### Nonholonomic Constraints

Also cause underactuation. i.e. cars can't move sideways $\rightarrow$ doesn't restrict its set of possible configurations, but does restrict possible $\ddot{q}$.

<br />
<br />

## Dynamics of a Simple Pendulum

Equation of motion for a pendulum (solved using Euler Lagrange):

$$ ml^2\ddot{\theta} + b\dot{\theta} + mgl\sin\theta = u_0 $$

### Overdamped Case

Pendulum is overdamped if $b\dot{\theta} >> ml^2\ddot{\theta}$; which is equivalent to $b \sqrt\frac{l}{g} >> ml^2$ ($\sqrt\frac{l}{g}$ is there to ensure both sides have the same units, and is assumed to have a reasonable value and therefore not affect the inequality relationship).

Then, the equation of motion becomes roughly this (1st order):

$$ b\dot{\theta} + mgl\sin\theta = u_0 $$

Now, plotting $\dot{\theta}$ vs $\theta$:

<center><img src="Media/overdamped.png" style="width:45%"/></center><br />

We see equilibria at $\theta=-\pi, 0, \pi$. The only stable equilibira is at $\theta=0$ (when $\theta$ becomes slightly larger, $\dot{\theta}$ is negative, and vice versa when $\theta$ becomes slightly smaller.)

Types of Lyapunov Stability; if $x^*$ is the stable point:
- **Locally Stable**: for some small scalar $\epsilon$, if $||x(0) - x^*|| < \epsilon$, then $\forall t ||x(t) - x^*|| < \epsilon$. Basically, for some ball, if you start in the ball, you will always remain in the ball.
    - Example: $\dot{x} = 0$
- **Locally Attractive**: if $||x(0) - x^*|| < \epsilon$, then $lim_{t \rightarrow \inf x(t)} = x^*$. Basically, $x$ will reach the stable point eventually.
- **Locally Asymptotically Stable**: If both **Locally Stable** & **Locally Attractive**
- **Locally Exponentially Stable**: if $||x(0) - x^*|| < \epsilon$, then if $||x(t) - x^*|| < Ce^{-\alpha t}$, for some $C, \alpha$.

Note: Lyapunov analysis only applies to passively dynamic systems (no external forcing).

Now, stable point drawn as block dots, unstable points drawn as circles:

<center><img src="Media/overdamped_stabilty.png" style="width:45%"/></center><br />

Notice, that anytime $\dot{x} > 0$, any initial condition will tend to move right, and vice versa for $\dot{x} < 0$. Therefore, we have **regions of attraction** around the black dots and **separatrix** (separating regions of attraction) at each white dot.

In certain cases, your system might have a parameter $w$, and varying $w$ varies the number of equilibirium points in the system. This is called a **bifurcation**. You can plot a **bifurcation diagram** of $x*$ versus $w$; it may have "pitches" if, at a certain $w$, the number of $x*$ changes.

### Underdamped Case

Now, $b=0$. $ ml^2\ddot{\theta} + mgl\sin\theta = u_0 $; there is no first order term.

We can rewrite the second order eqution as a first order equation with a change of variables (which doubles the number of variables). In general:

$$ \ddot{q} = f(q, \dot{q}, u) $$

becomes;

$$ \dot{x}_1 = x_2 $$
$$ ~~~~~~~~~~~~~~~\dot{x}_2 = f(x_1, x_2, u)$$

