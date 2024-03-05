# Lyapunov Analysis

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

Same, but with strict inequality.

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

Then $x$ will converge to the largest "invariant set" where $\dot{V}(x) = 0$. An "invariant set" is the set of $x$ where, once the system enters the set, it never leaves.

For example, for a simple pendulum, we can use the energy of the pendulum as the Lyapunov function. Then, $\dot{V}(x) \preceq 0$,, it's obvious that the only invariant sets for the pendulum are the fixed points; then the *largest invariant set* is the union of all the fixed points.