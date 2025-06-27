# VTOL Dynamics Model

using LinearAlgebra
using StaticArrays
using DifferentialEquations

@kwdef struct VTOL_3DOF{F}
    g::F = 9.81
    m::F = 11.0
    J::F = 1.135
    l::F = 0.5
    S::F = 0.55
    c::F = 0.18994
    rho::F = 1.225
    CD0::F = 0.043
    CDα::F = 0.03
    CDδ::F = 0.0135
    CDω::F = 0.1
    CL0::F = 0.23
    CLα::F = 5.61
    CLδ::F = 0.13
    CTp::F = 0.1
    CTf::F = 0.1
    CTr::F = 0.1
    CM0::F = 0.0135
    CMα::F = -2.74
    CMδ::F = -0.99
end

function dynamics(u, p, t)
    control = controller(u,p)
    return VTOL_3DOF_dynamics(u, control, p)
end

function controller(x,p)
    F = @SVector[0.0, 0.0] # Forces in body frame
    M = 0.0 # Moment in body frame
    u = control_allocation(F, M, x, p)
    return u
end

function control_allocation(F, M, x, p)
    # Unpack state variables
    px, pz, vx, vz, θ, q = x # X position, Z position, x-velocity, z-velocity, pitch angle, pitch rate
    α = atan(vz,vx) # Angle of attack
    Va = sqrt(vx^2 + vz^2) # True airspeed
    

end

function VTOL_3DOF_dynamics(x, u, p)
    # Unpack state variables
    px, pz, vx, vz, θ, q = x # X position, Z position, x-velocity, z-velocity, pitch angle, pitch rate
    α = atan(vz,vx) # Angle of attack
    Va = sqrt(vx^2 + vz^2) # True airspeed
    ωp, ωf, ωr, δe = u # pusher, front, rear, elevator

    R = @SMatrix[cos(θ) -sin(θ); sin(θ) cos(θ)] # Rotation matrix from inertial to body frame
    V = @SVector[vx, vz] # Velocity vector

    F, M = forces_and_moments(α, Va, u, p) # Forces and moments

    dpx, dpy = R' * V # North and East position derivatives
    dvx, dvz = 1/p.m*(R * @SVector[0.0, p.g] + F) - @SVector[q*vz, -q*vx] # North and East velocity derivatives
    dθ = q # Pitch angle derivative
    dq = M/p.J # Pitch rate derivative

    return @SVector[dpx, dpy, dvx, dvz, dθ, dq]
end

function forces_and_moments(α, Va, u, p)

    # Unpack control inputs
    ωp, ωf, ωr, δe = u

    R = @SMatrix[cos(α) -sin(α); sin(α) cos(α)] # Rotation matrix from wind to body frame

    # Aerodynamic forces
    D = 0.5 * p.rho * Va^2 * p.S * (p.CD0 + p.CDα * α^2) # Drag force
    D += p.CDω * (ωr + ωf) * Va
    L = 0.5 * p.rho * Va^2 * p.S * (p.CL0 + p.CLα * α + p.CLδ * δe) # Lift force

    # Control forces
    Fp = ωp^2 * p.CTp # Pusher thrust
    Ff = ωf^2 * p.CTf # Front thrust
    Fr = ωr^2 * p.CTr # Rear thrust

    # Total forces
    F = R * @SVector[-D, -L] + @SVector[Fp, -(Ff+Fr)] # Forces in body frame

    # Moments
    M = (Ff - Fr) * 2p.l + 0.5 * p.rho * Va^2 * p.S * p.c * (p.CM0 + p.CMα * α + p.CMδ * δe) # Pitch moment

    return F, M

end

