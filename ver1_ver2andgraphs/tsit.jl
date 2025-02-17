using Sockets
using DifferentialEquations
using StaticArrays
using Plots
using Dates
using LinearAlgebra
using CSV
using DataFrames

const σ = 10.0
const ρ = 28.0
const β = 8/3
const k = 5.0
const SYNC_THRESHOLD = 0.01
const INITIAL_DELAY_STEPS = 12

function run_synchronization_tsit5()
    packets_received = 0
    sync_achieved = false
    sync_time = nothing
    start_time = now()
    connection_time = nothing
    data_reception_time = nothing

    function lorenz_slave!(dy, y, p, t)
        σ_local, ρ_local, β_local, x, u = p
        dy[1] = σ_local * (y[2] - y[1]) + u[1]
        dy[2] = y[1] * (ρ_local - y[3]) - y[2] + u[2]
        dy[3] = y[1] * y[2] - β_local * y[3] + u[3]
    end

    function backstepping_control(x, y, p)
        σ_local, ρ_local, β_local = p
        e = y .- x
        u1 = -σ_local * (e[2] - e[1]) + e[2]
        u2 = -ρ_local * e[1] + e[2] + (y[1] * y[3]) - (x[1] * x[3]) + e[3]
        u3 = (-y[1] * y[2]) + (x[1] * x[2]) + β_local * e[3] - (3 + 2k) * e[1] - (5 + 2k) * e[2] - (3 + k) * e[3]
        return SVector(u1, u2, u3), e
    end

    y0 = [5.0, 5.0, 5.0]
    dt = 0.001
    T = 100.0
    t_eval = 0:dt:T
    
    println("Slave (Tsit5): Connecting to master...")
    
    while true
        client = try
            connection_start = now()
            socket = connect("127.0.0.1", 2000)
            connection_time = now() - connection_start
            socket
        catch e
            println("Slave (Tsit5): Connection failed. Retrying...")
            sleep(2)
            continue
        end

        master_x = []
        data_reception_start = now()
        while true
            line = try
                readline(client)
            catch e
                break
            end
            if isempty(line)
                break
            end
            packets_received += 1
            x = parse.(Float64, split(line, ","))
            push!(master_x, x)
        end
        data_reception_time = now() - data_reception_start
        close(client)

        if isempty(master_x)
            return
        end
        
        master_x = hcat(master_x...)'
        n_steps = length(t_eval)
        y_traj = zeros(3, n_steps)
        e_traj = zeros(3, n_steps)
        y_current = copy(y0)
        sync_phases = zeros(Int, n_steps)
        simulation_start = now()
        
        for (i, t) in enumerate(t_eval)
            x = master_x[i, :]
            
            if i <= INITIAL_DELAY_STEPS
                u = zeros(3)
                e = y_current .- x
            else
                u, e = backstepping_control(x, y_current, (σ, ρ, β))
            end
            
            prob = ODEProblem(lorenz_slave!, y_current, (t, t+dt), (σ, ρ, β, x, u))
            sol = solve(prob, Tsit5())
            y_current = sol.u[end]
            y_traj[:, i] = y_current
            e_traj[:, i] = e
            
            error_norm = norm(e)
            if !sync_achieved && error_norm < SYNC_THRESHOLD
                sync_achieved = true
                sync_time = now() - start_time
            end
            
            if i <= INITIAL_DELAY_STEPS
                sync_phases[i] = 0
            elseif error_norm < SYNC_THRESHOLD
                sync_phases[i] = 2
            else
                sync_phases[i] = 1
            end
        end
        simulation_time = now() - simulation_start

        for j in 1:3
            p1 = plot(t_eval, [master_x[:, j] y_traj[j, :]], 
                    label=["Master" "Slave (Tsit5)"],
                    xlabel="Time", ylabel="State")
            title!(p1, "Tsit5 Master-Slave Comparison: Component $("xyz"[j])")
            savefig(p1, "tsit5_comparison_$("xyz"[j]).png")
            
            p2 = plot(t_eval, e_traj[j, :], 
                    label="Error (Tsit5)",
                    xlabel="Time", ylabel="Error")
            title!(p2, "Tsit5 Error: Component $("xyz"[j])")
            savefig(p2, "tsit5_error_$("xyz"[j]).png")
        end

        timing_df = DataFrame(
            metric = ["Connection Time", "Data Reception Time", "Sync Time", "Simulation Time", "Total Time"],
            duration = [connection_time, data_reception_time, sync_time, simulation_time, now() - start_time]
        )
        CSV.write("tsit5_timing.csv", timing_df)

        df = DataFrame(
            time = collect(t_eval),
            error_x = e_traj[1, :],
            error_y = e_traj[2, :],
            error_z = e_traj[3, :],
            sync_phase = sync_phases
        )
        CSV.write("tsit5_data.csv", df)
        
        break
    end
end

run_synchronization_tsit5()