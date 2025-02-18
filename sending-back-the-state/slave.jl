using Sockets
using DifferentialEquations
using StaticArrays
using Plots

using LinearAlgebra


function run_synchronization()
    function lorenz_slave!(dy, y, p, t)
        σ, ρ, β, u = p  
        
        dy[1] = σ * (y[2] - y[1]) + u[1]
        dy[2] = y[1] * (ρ - y[3]) - y[2] + u[2]
        dy[3] = y[1] * y[2] - β * y[3] + u[3]
    end

    function backstepping_control(x, y, p)
        σ, ρ, β = p
        e = y .- x  
        
        u1 = -σ * (e[2] - e[1]) + e[2]
        u2 = -ρ * e[1] + e[2] + (y[1] * y[3]) - (x[1] * x[3]) + e[3]
        u3 = (-y[1] * y[2]) + (x[1] * x[2]) + β * e[3] - (3 + 2k) * e[1] - (5 + 2k) * e[2] - (3 + k) * e[3]
        
        return SVector(u1, u2, u3), e
    end
    k = 5.0  
    p = (10.0, 28.0, 8 / 3)
    y0 = [5.0, 5.0, 5.0]  
    dt = 0.001
    T = 7.0
    t_eval = 0:dt:T
    println("Slave: Connecting to master...")
    
    while true
        client = try
            connect("127.0.0.1", 2000)
        catch
            println("Slave: Connection failed. Retrying...")
            sleep(2)
            continue
        end
        println("Slave: Connected to master.")

        master_x = []
        while true
            line = try
                readline(client)
            catch
                ""
            
            end
            if isempty(line) || line == "END"
                break
            end

            x = try
                parse.(Float64, split(line, ","))
            catch
                println("Slave: Error parsing data.")
                continue
            end
            push!(master_x, x)
        end
    
        if isempty(master_x)
            println("Slave: No data received. Exiting.")
            return
        end
        master_x = hcat(master_x...)'
        n_steps = length(t_eval)
        y_traj = zeros(3, n_steps)
        e_traj = zeros(3, n_steps)
        y_current = copy(y0)
        lowest = Inf
        minarg = -1
        for (i, t) in enumerate(t_eval)
            x = master_x[i, :] 
            u, e = backstepping_control(x, y_current, p)
            
            prob_slave = ODEProblem(lorenz_slave!, y_current, (t, t+dt), (p..., u))
            sol_slave = solve(prob_slave, Tsit5(), dtmax=dt)
            y_current = sol_slave.u[end]
            y_traj[:, i] = y_current
            e_traj[:, i] = e
            if norm(e) < lowest
                lowest = norm(e)
                minarg = i
            end
        end

        println("last", e_traj[:, end], norm(e_traj[:, end]))
        y0 = y_traj[:, end]
        write(client, join(y0, ",") * "\n")
        flush(client)
        prob_slave = ODEProblem(lorenz_slave!, y0, (7.0, 100.0), (p..., [0.0, 0.0, 0.0]))
        sol_slave = solve(prob_slave, Tsit5(), dtmax=dt)
        y_traj1 = sol_slave.u
        final_master_x = []
        while true
            line = try
                readline(client)
            catch
                ""
            end

            if isempty(line) || strip(line) == "END"
                println("Slave: Received final end of transmission.")
                break
            end

            x = try
                parse.(Float64, split(line, ","))
            catch
                println("Slave: Error parsing data.")
                continue
            end
            push!(final_master_x, x)
        end
        close(client)
        final_master_x = hcat(final_master_x...)'
        y_traj1 = hcat(y_traj1...)'
        println("Slave: Received final master state.")
        e_traj1 = final_master_x .- y_traj1
        for j in 1:3
            p1 = plot(t_eval, [master_x[:, j] y_traj[j, :]], 
                 label=["Master $("xyz"[j])" "Slave $("xyz"[j])"],
                 xlabel="Time", ylabel="State", 
                 title="Master-Slave Trajectories: Component $("xyz"[j])")
            savefig(p1, "trajectory$("xyz"[j]).png")
            
            p2 = plot(t_eval, e_traj[j, :], 
                 label="Error $("xyz"[j])", 
                 xlabel="Time", ylabel="Error",
                 title="Synchronization Error: Component $("xyz"[j])")
            savefig(p2, "error$("xyz"[j]).png")

            p3 = plot(sol_slave.t, [final_master_x[:, j] y_traj1[:, j]], 
                 label=["Master $("xyz"[j])" "Slave $("xyz"[j])"],
                 xlabel="Time", ylabel="State", 
                 title="Master-Slave Trajectories: Component $("xyz"[j])")
            savefig(p3, "final_trajectory$("xyz"[j]).png")

            p4 = plot(sol_slave.t, e_traj1[:, j], 
                 label="Error $("xyz"[j])", 
                 xlabel="Time", ylabel="Error",
                 title="Synchronization Error: Component $("xyz"[j])")
            savefig(p4, "final_error$("xyz"[j]).png")
        end

        println("All plots have been saved.")
        break
    end
end 
run_synchronization()
