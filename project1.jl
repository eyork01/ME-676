# project1.jl
# Author: Ricardo Gutierrez, Madison Vogt, Ethan York
# University of Kentucky
# ME 676, Dr. Poonawala
# February 17, 2023

include("startup.jl")
include("project1_functions.jl")
using LinearAlgebra
using RigidBodyDynamics

# Define Variables
path1 = path(mechanism, bodies(mechanism)[1], bodies(mechanism)[end])
e = 0.1                         # Max error term
desiredX = request_input()      # Calls custom input function

# Initialize q
global q = configuration(state)
println("Initial q: ",q)

# Main Loop
for i = 0:100
    # Calculate new delta X and Jacobian
    local currentX = vcat(calcXinW(state,ee_point).v.data...)
    local dx = -(currentX-desiredX)
    println("ΔX ",i," : ", dx)
    local J = Matrix(point_jacobian(state, path1, transform(state, ee_point, base_frame)))[1:3,:]

    # Exit loop if end-effector is in position
    (norm(dx) < e) && break
    
    # Calculate new joint angles
    local dq = pinv(J) * dx * 0.1
    global q = q + dq

    # Update robot arm
    update_planar_state(state,mvis,q)                                                       
    sleep(0.1)  # Add small delay for animation
end



#= NOTES
ef2 = findbody(robot, "endeffector")
base_frame = default_frame(bodies(robot)[1])
#global J = Matrix(geometric_jacobian(state, path1))[1:3,:]   #Jp = point_jacobian(state, p, transform(state, point, world))
desiredX = vcat(Point3D(base_frame,xin[1],xin[2],xin[3]).v.data...)
=#