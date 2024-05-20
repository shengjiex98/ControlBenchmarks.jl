module ControlBenchmarks

export benchmarks
export delay_lqr, pole_place, augment

using ControlSystemsBase
using LinearAlgebra

# Resistor-capacitor network
sys_rcn = let
	r_1 = 100000
	r_2 = 500000
	r_3 = 200000
	c_1 = 0.000002
	c_2 = 0.000010
	A = [-1/c_1 * (1/r_1 + 1/r_2)  1/(r_2*c_1)
	     1/(r_2*c_2)               -1/c_2 * (1/r_2 + 1/r_3)]
	B = [1/(r_1*c_1)
	     1/(r_3*c_2)]
	# C = [1  -1]
    C = I
	D = 0

	ss(A, B, C, D)
end

# F1-tenth car
sys_f1t = let 
    v = 6.5
    L = 0.3302
    d = 1.5
    A = [0 v ; 0 0]
    B = [0; v/L]
    C = [1 0]
    D = 0

    ss(A, B, C, D)
end

# DC motor
sys_dcm = let
    A = [-10 1; -0.02 -2]
    B = [0; 2]
    C = [1 0]
    D = 0

    ss(A, B, C, D)
end

# Car suspension system
sys_css = let
    A = [0. 1 0 0; -8 -4 8 4; 0 0 0 1; 80 40 -160 -60]
    B = [0; 80; 20; -1120]
    C = [1 0 0 0]
    D = 0

    ss(A, B, C, D)
end

# Electronic wedge brake
sys_ewb = let
    A = [0 1; 8.3951e3 0]
    B = [0; 4.0451]
    C = [7.9920e3 0]
    D = 0

    ss(A, B, C, D)
end

# Cruise control 1
sys_cc1 = let
    A = -0.05
    B = 0.01
    C = 1
    D = 0

    ss(A, B, C, D)
end

# Cruise control 2
sys_cc2 = let
    A = [0 1 0; 0 0 1; -6.0476 -5.2856 -0.238]
    B = [0; 0; 2.4767]
    C = [1 0 0]
    D = 0

    ss(A, B, C, D)
end

sys_mpc = ss(tf([3, 1],[1, 0.6, 1]))

benchmarks = Dict([
    :RCN => sys_rcn,
    :F1T => sys_f1t,
    :DCM => sys_dcm,
    :CSS => sys_css,
    :EWB => sys_ewb,
    :CC1 => sys_cc1,
    :CC2 => sys_cc2,
    :MPC => sys_mpc
])

"""
delay_lqr(sys, h; Q=I, R=I)

LQR controller design of a delayed discrete state-space model. Q and R matrices
are default to be identity matrices.
"""
function delay_lqr(sys::AbstractStateSpace{<:Continuous}, h::Float64; Q=I, R=I)
    sysd_delay = c2d(sys, h) |> augment
    lqr(sysd_delay, Q, R)
end

"""
pole_place(sys, h; p=0.9)

Pole placement controller design of a delayed discrete state-space model. Note
this only works for single input single output (SISO) systems. I.e., y and u 
must have dimensionality of 1.
"""
function pole_place(sys::AbstractStateSpace{<:Continuous}, h::Float64; p=0.9)
    sysd_delay = c2d(sys, h) |> augment
    place(sysd_delay, vcat([0], fill(p, size(sys.A)[1])))
end

"""
augment(sysd)

Augment the discrete state-space model `sysd` with a one-period delay
"""
function augment(sysd::AbstractStateSpace{<:Discrete})
    p = size(sysd.A, 1)
    q = size(sysd.B, 2)
    r = size(sysd.C, 1)
    A = [sysd.A sysd.B; zeros(q, p+q)]
    B = [zeros(p, q); I]
    C = [sysd.C zeros(r, q)]
    D = sysd.D
    ss(A, B, C, D, sysd.Ts)
end

end
