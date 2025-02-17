using DifferentialEquations
using Plots
using StaticArrays
using Sockets
using Dates

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

# Packet counter
packets_sent = 0

function lorenz_master!(dx, x, p, t)
    dx[1] = σ * (x[2] - x[1])
    dx[2] = x[1] * (ρ - x[3]) - x[2]
    dx[3] = x[1] * x[2] - β * x[3]
end

# Solve using Tsit5
println("Solving master system...")
prob_master = ODEProblem(lorenz_master!, x0, tspan)
sol_master = solve(prob_master, Tsit5(), saveat=dt)

function run_master()
    server = listen(2000)
    println("Master: Server started. Waiting for connections...")
    start_time = now()

    while true
        client = accept(server)
        println("Master: Slave connected at $(now() - start_time)")
        
        global packets_sent = 0
        
        # Intentionally delay first 6 timesteps to show unsynchronized behavior
        delay_steps = 6
        for i in 1:delay_steps
            sleep(0.1)  # Small delay
        end
        
        for i in t_eval
            x = sol_master(i)
            write(client, join(x, ",") * "\n")  # Convert to CSV format
            flush(client)
            global packets_sent += 1
        end

        println("Master: Statistics:")
        println("Total packets sent: $packets_sent")
        println("Total time running: $(now() - start_time)")
        
        close(client)
        break
    end

    close(server)
end

run_master()