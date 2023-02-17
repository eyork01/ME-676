# project1_functions.jl
# Author: Ricardo Gutierrez, Madison Vogt, Ethan York
# University of Kentucky
# ME 676, Dr. Poonawala
# February 17, 2023

function update_planar_state(state,mvis,q)
    set_configuration!(state, q)
    set_configuration!(mvis, configuration(state))
end

# User Input Function (MUST RUN PROGRAM FROM TERMINAL) # include("project1.jl")
function request_input()
    println("Please enter your 3D coordinates \n")
    prompts = ["X: ","Y: ","Z: "]
    xin = zeros(3)
    for i = 1:3
        print(prompts[i])
        xin[i] = parse(Float64, readline())
    end
    println()
    return xin
end

# Returns end-effector frame position in world frame
function calcXinW(state,x)
    mechanism = state.mechanism;
    ee_frame=default_frame(bodies(mechanism)[end])
    base_frame = default_frame(bodies(mechanism)[1])
    # Obtain the transformation from world to EE
    EEinBase = relative_transform(state,base_frame,ee_frame)
    x_in_w = inv(EEinBase)*x
    return x_in_w
end

# Load mechanism and visualize at q=0 config
delete!(vis)
mvis, mechanism = display_urdf("project1Robot.urdf",vis)
state = MechanismState(mechanism)
update_planar_state(state,mvis,[0,0,0,0,0,0,0,0,0,0,0])

# Create base and end-effector frames:
ee_body = findbody(mechanism, "endeffector2")
ee_frame = default_frame(ee_body)
ee_point = Point3D(default_frame(ee_body), 0, 0, 0)
base_frame = default_frame(bodies(mechanism)[1])