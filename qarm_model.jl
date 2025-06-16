using ModelingToolkit

#define time
@parameters t

#define joint variables(symbolic)
@variables θ1(t) θ2(t) θ3(t) θ4(t)
@variables w1(t) w2(t) w3(t) w4(t) #angular velocity
@variables τ1(t) τ2(t) τ3(t) τ4(t) #torques

D = Differential(t)

#define dummy physical parameters
@parameters I1 I2 I3 I4 #movement of inertia

#equations: rotational dynamics for each joint(τ = I * α)
eqs = [
    D(θ1) ~ w1,
    D(w1) ~ τ1/τ1,

    D(θ2) ~ w2,
    D(w2) ~ τ2/τ2,

    D(θ3) ~ w3,
    D(w3) ~ τ3/τ3,

    D(θ4) ~ w4,
    D(w4) ~ τ4/τ4,
]

#build the system
@named qarm = ODESystem(eqs, t)

#forward kinematics
@parameters L1 L2 L3 L4

#forward kinematics equations(simplified 2D+ vertical Z joint lift)
#base at origin, cumulative transformation
x_pos = L1*cos(θ1) + L2*cos(θ1 + θ2) + L3*cos(θ1 + θ2 + θ3)
y_pos = L1*sin(θ1) + L2*sin(θ1 + θ2) + L3*sin(θ1 + θ2 + θ3)
z_pos = L4*sin(θ4) #optical vertical actuator motion

#grouping them
position = [x_pos,y_pos,z_pos]
println("end effector position (symbolic):", position)