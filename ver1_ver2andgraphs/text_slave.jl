using Sockets
using LinearAlgebra
using StaticArrays

function run_slave()
    function lorenz_slave(y, x, p, u)
        σ, ρ, β = p
        dy = zeros(3)
        dy[1] = σ * (y[2] - y[1]) + u[1]
        dy[2] = y[1] * (ρ - y[3]) - y[2] + u[2]
        dy[3] = y[1] * y[2] - β * y[3] + u[3]
        return dy
    end

    function backstepping_control(x, y, p, k)
    # Errors
        σ, ρ, β= p
        e1 = y[1] - x[1]
        e2 = y[2] - x[2]
        e3 = y[3] - x[3]

        u1 = -σ * (y[2] - y[1] - x[2] + x[1]) + e2
        u2 = -ρ * (y[1] - x[1]) + (y[2] - x[2]) + (y[1] * y[3]) - (x[1] * x[3]) + e3
        u3 = (-y[1] * y[2]) + (x[1] * x[2]) + (β * (y[3] - x[3])) - ((3 + (2 * k[1])) * e1) - ((5 +( 2 * k[1])) * e2) - ((3 + k[1]) * e3)
    
        return SVector(u1, u2, u3), SVector(e1, e2, e3)
    end

    dt = 0.01
    T = 10.0
    n_steps = Int(T / dt)
    p = (10.0, 28.0, 8 / 3)
    k = (5.0, 5.0, 5.0)  

    y = [5.0, 5.0, 5.0]

    println("Slave: Connecting to master...")
    client = nothing  # Define `client` outside the loop to avoid scope issues
    while true
        try
            client = connect("127.0.0.1", 2000)
            break  # Exit the loop once connected
        catch
            println("Slave: Connection failed. Retrying...")
            sleep(2)  
        end
    end

    println("Slave: Connected to master.")

    for i in 1:n_steps
        line = try
            readline(client)
        catch
            ""
        end

        if isempty(line)
            println("Slave: Received empty data.")
            continue
        end

        x = try
            parse.(Float64, split(line, ","))
        catch
            println("Slave: Error parsing data.")
            continue
        end

        u_control, e = backstepping_control(x, y, p, k)

        y += lorenz_slave(y, x, p, u_control) * dt
    end

    # Calculate the mask based on y_slave state (same as master)
    println(y)
    mask = round(Int, sum(y))  # Ensure this is the same formula as the master

    # Receive encrypted message from master
    println("Slave: Receiving encrypted message from master...")
    encrypted_message = ""

    try
        encrypted_message = readline(client)
        println("Slave: Encrypted message received: $encrypted_message")  # Debug output
    catch
        println("Slave: Error receiving message.")
    end

    # Check if the encrypted message is not empty
    if isempty(encrypted_message)
        println("Slave: Received empty message, cannot decrypt.")
    else
        # Decrypt the message using the same mask
        encrypted_values = try
            parse.(Int, split(encrypted_message, ","))
        catch
            println("Slave: Error parsing encrypted message.")
            return
        end

        # Decrypt using XOR with the mask
        decrypted_message = join(Char(c ⊻ mask) for c in encrypted_values)
        println("Slave: Decrypted message: $decrypted_message")
    end
    println("mask: $mask")
    close(client)  # Ensure client is closed after communication
end

run_slave()
