using Sockets

function run_master()
    # Define the Lorenz system
    function lorenz(x, p)
        σ, ρ, β = p
        dx = zeros(3)
        dx[1] = σ * (x[2] - x[1])
        dx[2] = x[1] * (ρ - x[3]) - x[2]
        dx[3] = x[1] * x[2] - β * x[3]
        return dx
    end

    # Simulation parameters
    dt = 0.01
    T = 10.0
    n_steps = Int(T / dt)
    p = (10.0, 28.0, 8 / 3)

    # Initial state of the master system
    x_master = [1.0, 1.0, 1.0]

    # Start server
    server = listen(2000)
    println("Master: Server started. Waiting for connections...")
    client = accept(server)
    println("Master: Slave connected.")

    # Synchronize with slave
    for _ in 1:n_steps
        # Simulate master dynamics
        x_master += lorenz(x_master, p) * dt

        # Send the state to slave
        write(client, join(x_master, ",") * "\n")
        sleep(dt)
    end

    println("Master: Finished sending synchronization data.")
    println(sum(x_master))
    # Calculate the mask based on x_master state
    mask = round(Int, sum(x_master))  # Ensure this is the same formula as the slave

    # Encrypt and send a message using XOR
    message = "Hello from Master"
    encrypted_message = join([Int(c) ⊻ mask for c in message], ",")
    write(client, encrypted_message)
    println("Master: Sending encrypted message...")
    println("Message: ", encrypted_message)
    sleep(2)
    println(x_master)
    println("mask: $mask")
    close(client)
    close(server)
end

run_master()
