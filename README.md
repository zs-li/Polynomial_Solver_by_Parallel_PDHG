# Polynomail Solver by Parallel PDHG

This repository is a solver for Polynomial Optimization of the following form:

```math
\begin{align}
    \min _{\mathbf{P}(t)} \int_{0}^T \mathbf{P}(t)^{\top} \boldsymbol{\Sigma} \mathbf{P}(t)& +\mathbf{\mu}(t)^\top \mathbf{P}(t) {\rm d} t \\
    \text { s.t. } \quad L_{k} \mathbf{P}(t)-b_{k} &  \geq 0 ,\ t\in[0,T],\ k=1, \cdots, N_{\text{ineq }} \\
    S_{ k} \mathbf{P}(t_k)-h_{k} & =0,\ k=1, \cdots, N_{\text{eq }}
\end{align}
```

where $\mathbf{P}(t)$ is the polynomial vector of the following form

```math
\mathbf{p}(t)=\begin{matrix}
			\begin{bmatrix} p_1(t) \\ \vdots \\ p_m(t) \end{bmatrix}
		\end{matrix},\
		\mathbf{P}(t)=\begin{bmatrix} \mathbf{p}^{(0)}(t) \\ \vdots \\ \mathbf{p}^{(s)}(t)\end{bmatrix}
```

with $p_i(t)$ as a polynomial with one variable $t$ and maximum degree $d$. $\mathbf{p}^{(s)}(t)$ is $s$-th order derivative of $\mathbf{p}(t)$.
$N_{\rm ineq}$ and $N_{\rm eq}$ are number of polynomial inequality and equality constraints.
$\mu(t):\mathbb{R}\rightarrow\mathbb{R}^{m(s+1)}$ is a known function and only vectors of polynomials are supported for now.

The solver is written in [Julia](https://julialang.org/). The example interface of our proposed solver is as follows:

```julia
model = PPPS.Model() # PPPS is Parallel PDHG Polynomial Solver
@variable(model, p, dim=m, deg=s, seg=N)# polynomial variable
for k in 1:Nineq
    @ineqconstraint(model, Lk, bk,T ) # poly ineq parameters
end
for k in 1:Neq
    @eqconstraint(model, Sk, hk, tk)# poly eq parameters
end
@objective(model, Min, Σ, µ) # µ is polynomial parameters 
optimize!(model)
```

## Translating and solving process in the background

This is the paper [link](https://arxiv.org/abs/2303.17889) of original work.

The polynomial optimization problem is translated into an SDP problem and solved with customized primal-dual based iterations in a parallelized manner.
This solver is originally proposed to solve a Model Predictive Control (MPC) problem of a continuous-time linear time-invariant system subject to continuous-time path constraints on the states and the inputs. By leveraging the concept of differential flatness, we can replace the differential equations governing the system with linear mapping between the states, inputs, and flat outputs (including their derivatives). The flat outputs are then parameterized by piecewise polynomials, and the model predictive control problem can be equivalently transformed into a Semi-Definite Programming (SDP) problem via Sum-of-Squares (SOS), ensuring constraint satisfaction at every continuous-time interval. We further note that the SDP problem contains a large number of small-size semi-definite matrices as optimization variables. To address this, we develop a Primal-Dual Hybrid Gradient (PDHG) algorithm that can be efficiently parallelized to speed up the optimization procedure. 

## Example on solving continuous-time MPC problem

We use the quad-tank process as an example of the usage of our solver.





## Result

![Visualiztion of the optimziation process](https://github.com/zs-li/MPC_PDHG/blob/main/anim.gif)

This figure shows the optimization intermediate states of our proposed algorithm. As optimization goes on, the polynomials representing the state trajectories are converging to the actual trajectories governed by system dynamics and subject to constraints. 

"k" in the title is the number of apply steps, also the index of sequential SDP problems we solve.

The shift warm start strategy can speed up the optimization process by setting the initial values in the overlapped horizon as in the previous step. Only the final segment (**new** horizon) is needed to be calculated from a random intial value.

## package requirements

The package we use for code test are:

```
NumericalIntegration v0.3.3
TensorOperations v3.2.4
CUDA v3.13.1
```

Auxiliary code for visualization are:

```
DynamicPolynomials v0.4.6
Plots v1.38.8
```

For compatibility constraints, ```TensorOperations``` requires that ```CUDA``` version \< v4.1.4.
