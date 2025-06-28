using Plots
using LinearAlgebra
using StaticArrays

function animate_vtol(sol; z_offset=5.0,  fps = 30, filename = "vtol_animation.gif")
    t_hist = range(sol.t[1], sol.t[end], length = round(Int, fps * (sol.t[end] - sol.t[1])))

    # Geometry and scaling
    body_length = 0.6
    body_width = 0.1
    arm_length = 0.3
    thrust_scale = 0.1
    vel_scale = 0.5
    prop_offset = SVector(0.4, 0.0)

    anim = @animate for t in t_hist
        state = sol(t)
        # v = state[1]     # horizontal position
        # z = -state[2]     # vertical position (altitude)
        # vx_body = state[3]
        # vz_body = state[4]
        # θ = state[5]

        # u = @SVector [0.0, 0.0, 0.0, 0.0]
        # T_left, T_right, T_prop, M_elev = u

        # # Rotation matrix: body to world
        # R = SMatrix{2,2}(cos(θ), -sin(θ), sin(θ), cos(θ))

        # # Convert body-frame velocity to world-frame
        # vel_world = R * SVector(vx_body, vz_body)
        # vel_world = SVector(vel_world[1], -vel_world[2])

        v = state[1]     # horizontal position
        z = state[3] + z_offset     # vertical position (altitude)
        vx = state[2]
        vz = state[4]
        θ = state[5]

        u = @SVector [0.0, 0.0, 0.0, 0.0]
        T_left, T_right, T_prop = state[7:9]

        # Rotation matrix: body to world
        R = SMatrix{2,2}(cos(θ), sin(θ), -sin(θ), cos(θ))

        # Convert body-frame velocity to world-frame
        vel_world = SVector(vx, vz)

        # Body shape
        body = [
            SVector(-body_length/2, -body_width/2),
            SVector( body_length/2, -body_width/2),
            SVector( body_length/2,  body_width/2),
            SVector(-body_length/2,  body_width/2),
            SVector(-body_length/2, -body_width/2)
        ]
        body_world = [R * p .+ SVector(v, z) for p in body]
        bx = [p[1] for p in body_world]
        by = [p[2] for p in body_world]

        # Rotor and propeller positions
        left_pos  = R * SVector(-arm_length, 0.0) .+ SVector(v, z)
        right_pos = R * SVector( arm_length, 0.0) .+ SVector(v, z)
        prop_pos  = R * prop_offset .+ SVector(v, z)

        # Thrust directions in world frame
        thrust_y = R * SVector(0.0, 1.0)
        thrust_x = R * SVector(1.0, 0.0)

        # Arrows
        left_arrow  = thrust_scale * T_left * thrust_y
        right_arrow = thrust_scale * T_right * thrust_y
        prop_arrow  = thrust_scale * T_prop * thrust_x
        vel_arrow   = vel_scale * vel_world

        # Plot
        plot(bx, by, lw=2, label="", aspect_ratio=1, xlims=(-5,5), ylims=(0,12))
        scatter!([left_pos[1], right_pos[1], prop_pos[1]], [left_pos[2], right_pos[2], prop_pos[2]], label="", color=:black)

        # Thrust arrows
        quiver!([left_pos[1]], [left_pos[2]], quiver=([left_arrow[1]], [left_arrow[2]]), color=:red, label="")
        quiver!([right_pos[1]], [right_pos[2]], quiver=([right_arrow[1]], [right_arrow[2]]), color=:red, label="")
        quiver!([prop_pos[1]], [prop_pos[2]], quiver=([prop_arrow[1]], [prop_arrow[2]]), color=:orange, label="")

        # Velocity vector (center of mass)
        quiver!([v], [z], quiver=([vel_arrow[1]], [vel_arrow[2]]), color=:blue, label="")

        title!("VTOL, t = $(round(t, digits=2)) s")
    end

    gif(anim, filename, fps=fps)
end
