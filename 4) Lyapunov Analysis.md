# Lyapunov Analysis

What/why? Lyapunov is not an optimal control scheme; it is more of a "viable" control scheme--will you get to $x^*$ eventually or not. Comparing to Value Iteration:

$$ \dot{J}^*(x) = -\ell(x, u^*) $$
$$ \text{vs} $$
$$ \dot{V}(x) \prec 0 $$

We can see that HJB's result is a (specifically, the optimal) Lyapunov function (if $\ell(x, u)$ is positive definite). While HJB requires the function to take on a specific gradient value, the Lyapunov just requires that the function is decreasing.

## Lyapunov Functions ($V$)

Any positive definite function (denoted as $V \succ 0$) of system state that decreases (possibly weakly) with time as the system evolves.

### Lyapunov's Direct Method

Computes whether a system is locally stable, asymptotically stable, or exponentially stable (or none) using $V$.

#### Stable in the sense of Lyapunov

Given system dynamics $\dot{x} = f(x)$, and some small region/subset of $\R^N$ around the origin $D$, if you can produce $V(x)$ such that:

$$ V(x) > 0, ~~~\forall x \in D \backslash \{0\}~~~ V(0)=0 $$
<center>and</center>

$$ \dot{V}(x) = \frac{\delta V}{\delta x} f(x) \leq 0, ~~~\forall x \in D \backslash \{0\} ~~~\dot{V}(0)=0$$

Then $x=0$ is stable in the sense of Lyapunov.

What these two requirements mean, intuitively: the first states that $V(x)$ must be positive at all $x$ in $D$, and $0$ at $x=0$. The seconds states that $V(x)$ must be decreasing or zero for all $x$ in $D$, and $0$ at $x=0$. 

#### Asymptotically Stable

$$ \dot{V}(x) = \frac{\delta V}{\delta x} f(x) < 0, ~~~\forall x \in D \backslash \{0\} ~~~\dot{V}(0)=0$$


#### Exponentially Stable

$$ \dot{V}(x) = \frac{\delta V}{\delta x} f(x) \leq -\alpha V(x), ~~~\forall x \in D \backslash \{0\} ~~~\dot{V}(0)=0$$

for some $\alpha>0$. Also, the same, but with a requirement on the rate that $V(x)$ decreases over time.

### Lyapunov Analysis for Global Stability

Stability in the sense of Lyapunov is an inherently local notion (with the epsilon/delta balls), but asymptotic and exponential stability can apply globally.

Notation note: 
 - $g(x) \succ 0$ = positive definite function ($g(x) > 0$, $g(0) = 0$)
 - $g(x) \prec 0$ = negative definite function ($g(x) < 0$, $g(0) = 0$)
 - $g(x) \succeq 0$ = positive semi-definite function ($g(x) \geq 0$, $g(0) = 0$)

#### Global Asymptotic Stability

If:

$$ V(x) \succ 0$$
$$ \dot{V}(x) = \frac{\delta V}{\delta x} f(x) \prec 0$$
$$ V(x) \rightarrow \infty ~~\text{whenever}~~ ||x|| \rightarrow \infty$$

This last condition is called "radial instability".


#### Global Exponential Stability   

If we have global asymptotic stability and that:

$$ \dot{V}(x) \preceq -\alpha V(x) $$

In other words (where $x^*$ is the point around which the system is stable): 

$$\exists ~C, \alpha ~~s.t.~~ ||x(t)-x^*|| < Ce^{-\alpha t} ||x(0)-x^*||$$


### LaSalle's Invariance Principle

Makes it possible to state asymptotic stability even when $\dot{V}(x) \preceq 0$ instead of $\dot{V}(x) \prec 0$.

If:

$$ V(x) \succ 0 ~~~\text{and}~~~ \dot{V}(x) \preceq 0$$
$$ V(x) \rightarrow \infty ~~\text{whenever}~~ ||x|| \rightarrow \infty$$

Then $x$ will converge to the largest "invariant set" where $\dot{V}(x) = 0$. An "invariant set" is the set of $x$ where, once the system enters the set, it never leaves. Basically, if $\dot{V}(x)$ is only negative semidefinite instead of negative definite, LaSalle's states that the system will converge to anywhere in the largest invariant set, instead of just where $V(x)=0$.

ALTERNATE FORMULATION: If:

$$ V(x) \succ 0 ~~~\text{and}~~~ \dot{V}(x) \preceq 0$$

and if $x(t)$ is a bounded trajectory, $x(t)$ will converge to the largest "invariant set" where $\dot{V}(x) = 0$. (This formulation does not require radial unboundedness).

Consider the example of the simple (non-actuated) damped pendulum; we can use the energy of the pendulum as the Lyapunov function. Then, $V(x) \succ 0$ and $\dot{V}(x) \preceq 0$, and $x(t)$ is a bounded trajectory for a pendulum.

Then, LaSalle's states that the pendulum will converge to the largest invariant set; for the damped pendulum, this the set of all the fixed points; once you enter a fixed point ($\theta = k \pi, \dot{\theta}=0$), the pendulum will not leave it. So, the pendulum will converge to either the bottom or top position.


### Regions of Attraction

Region of attraction to $x^*$ = largest set $D \subseteq X $ for which $x(0) \in D \implies lim_{t \rightarrow \infty} x(t) = x^*$. Regions of attraction are invariant sets.

Lyapunov Invariant Set and Region of Attraction Theorem:


### Pendulum Example

Recall the dynamics of the pendulum: $ ml^2 \ddot{\theta} + mgl \sin\theta = u$.

The energy of the system is: $E(x) = \frac{1}{2} ml^2 \dot{\theta}^2 - mgl\cos(\theta)$.

Then, $\dot{E} = ml^2 \ddot{\theta} \dot{\theta} + mgl \sin{\theta} \dot{\theta} = u \dot{\theta}$.

If our goal is to stand the pendulum up, then $E^d = E(x^*) = mgl$.

Let's pick the Lyapunov function $V(x) = \frac{1}{2}\tilde{E}^2$, where $\tilde{E} = E(x) - E^d$. Clearly, $V(x) \succ 0$.

Then the derivative of the Lyapunov function is: $\dot{V}(x) = \tilde{E}*\dot{\tilde{E}} = \tilde{E} * \dot{E} = \tilde{E} * u \dot{\theta}$.

In order to get a system that converges to $E^d = mgl$, we need to pick some $u$ such that $\dot{V}(x) \prec 0$, and $\dot{V}(x^*)=0 $.

We pick $u=-k\dot{\theta}\tilde{E}$ (for some positive constant $k$). Then, $\dot{V}(x)=\tilde{E}*u \dot{\theta} = \tilde{E}*(-k\dot{\theta}\tilde{E})\dot{\theta} = -k\dot{\theta}^2\tilde{E}^2$. Clearly, $\dot{V}(x)$ is decreasing except when $E(x) = E^d$, when it is zero.

Therefore, we validate that this control policy will be successful in getting the system to converge to the standing position.

In this example, we had no systematic way to choose $u$; we simply picked something that would obviously make $\dot{V}(x)$ negative definite. Next, we look at more analytical ways to do this.


## Lyapunov as an Algorithm

### Pendulum Example

Inputs: 
 - pendulum dynamics
 - parameterize family of polynomial/trigonometric functions for the Lyapunov function

Output: Coefficients for the polynomial + certificate of stability $\forall x$.

Example poly/trig function family: $V = (a - bc_0 + cs_0\dot{\theta}_0 + ds_0^2 + ec_0^2 + f\dot{\theta}^2_0)$

### Computing Lyapunov Functions using Linear Programming

$\dot{x} = f(x)$

$V(x) = \sum_{j=0}^J \alpha_j \phi_j(x) = \alpha^T \phi(x)$

where each $\phi_j(x)$ is some nonlinear basis function (of which there are a total of $J$). We would typically manually select the basis functions based on prior knowledge about the dynamics of the system (i.e. a single pendulum might have $1$, $\theta$, $\cos \theta$, $\sin \theta$, $\cos^2 \theta$, $\theta^2$ as its basis functions).


Sample states $x_i$. Make Lyapunov function; pick $\alpha$ to satisfy all of these samples:

$$ \forall x_i ~~~~V(0) = 0, ~~~~V(x_i) > \epsilon x_i^Tx_i, ~~~~\dot{V}(0) = 0, ~~~~\dot{V}(x_i) = \frac{\delta V}{\delta x} \bigg|_{x=x_i} f(x_i)< -\epsilon x_i^Tx_i$$

Basically, we're adding constraints to the program that $V(0)$ and $\dot{V}(0)$ are $0$, $V(x)$ is positive definite and radially unbounded, and the derivative of $V(x)$ is negtive definite/strictly decreasing (to guarantee asymptotic stability).

Plugging in our parameterization of $V(x)$:

$$ V(0) = 0, ~~~~\alpha^T \phi(x_i) > \epsilon x_i^Tx_i, ~~~~\dot{V}(0) = 0, ~~~~\alpha^T \frac{\delta \phi}{\delta x} f(x_i) < -\epsilon x_i^Tx_i$$

We get linear constraints in terms of $\alpha$.

With just these linear constriant and no objective, an LP solder could return any feasible $\alpha$, so to ensure a more reasonable answer, we add the linear objective: $ min_\alpha | \dot{V}(x_i)+1|$ (try to make gradient of $V(x)$ 1).

This probably works (assuming enough samples $x_i$), but since we only validate the Lyapunov function constraints on finite # of samples $x_i$, no certificate of stability.



### Aside: Optimization

 - Linear Program (LP): linear cost ($c^Tx$), linear constraints ($Ax \leq b$)
 - Quadratic Program (QP): quadratic cost ($\frac{1}{2} x^TQx + c^Tx$), linear constraints ($Ax \leq b$)
 - 2nd Order Cone Program (SOCP): quadratic cost ($\frac{1}{2} x^TQx + c^Tx$), conical constraints ($Ax \leq b$) ?????
 - Semi-deifinite Program (SDP): linear cost ($c^Tx$), linear constraints ($Ax \leq b$) + P.S.D matrix constraints ?????