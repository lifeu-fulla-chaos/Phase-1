using DifferentialEquations
using Plots
using StaticArrays
using Sockets

# System parameters - global constants
const σ = 10.0
const ρ = 28.0
const β = 8/3
const k = 5.0  # Backstepping gain

# Simulation parameters - global constants
const dt = 0.001
const T = 100.0
const tspan = (0.0, T)
const t_eval = 0:dt:T
x0 = [1.0, 1.0, 1.0]

function lorenz_master!(dx, x, p, t)
    dx[1] = σ * (x[2] - x[1])
    dx[2] = x[1] * (ρ - x[3]) - x[2]
    dx[3] = x[1] * x[2] - β * x[3]
end

prob_master = ODEProblem(lorenz_master!, x0, tspan)
sol_master = solve(prob_master, Tsit5(), saveat=dt)


function run_master()
    server = listen(2000)
    println("Master: Server started. Waiting for connections...")

    while true
        client = accept(server)
        println("Master: Slave connected.")

        for i in t_eval
            x = sol_master(i)
            write(client, join(x, ",") * "\n")  # Convert to CSV format
            flush(client)
        end

        println("Master: Finished sending state data to slave.")
        close(client)
        break
    end

    close(server)
end

run_master()