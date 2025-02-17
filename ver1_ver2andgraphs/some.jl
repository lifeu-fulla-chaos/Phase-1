using Plots
using CSV
using DataFrames
using Statistics
using Dates

println("Loading simulation data...")

# Load Tsit5 data with error handling
tsit5_data = try
    CSV.read("tsit5_data.csv", DataFrame)
catch e
    println("Error loading tsit5_data.csv: ", e)
    nothing
end

tsit5_timing = try
    CSV.read("tsit5_timing.csv", DataFrame)
catch e
    println("Error loading tsit5_timing.csv: ", e)
    nothing
end

# Load Euler data with error handling
euler_data = try
    CSV.read("euler_data.csv", DataFrame)
catch e
    println("Error loading euler_data.csv: ", e)
    nothing
end

euler_timing = try
    CSV.read("euler_timing.csv", DataFrame)
catch e
    println("Error loading euler_timing.csv: ", e)
    nothing
end

println("\nTiming Comparison:")

if !isnothing(tsit5_timing)
    println("Tsit5:")
    for row in eachrow(tsit5_timing)
        println("$(row.metric): $(row.duration)")
    end
else
    println("Tsit5 timing data not available")
end

if !isnothing(euler_timing)
    println("\nEuler:")
    for row in eachrow(euler_timing)
        println("$(row.metric): $(row.duration)")
    end
else
    println("\nEuler timing data not available")
end

# Only create comparison plots if both datasets are available
if !isnothing(tsit5_data) && !isnothing(euler_data)
    for (i, comp) in enumerate(["x", "y", "z"])
        p = plot(tsit5_data.time,
                [tsit5_data[!, "error_$comp"] euler_data[!, "error_$comp"]],
                label=["Tsit5" "Euler"],
                xlabel="Time",
                ylabel="Error",
                title="Error Comparison for Component $comp")
        savefig(p, "method_comparison_$comp.png")
    end
    
    error_stats = DataFrame(
        method = ["Tsit5", "Euler"],
        mean_error_x = [mean(abs.(tsit5_data.error_x)), mean(abs.(euler_data.error_x))],
        mean_error_y = [mean(abs.(tsit5_data.error_y)), mean(abs.(euler_data.error_y))],
        mean_error_z = [mean(abs.(tsit5_data.error_z)), mean(abs.(euler_data.error_z))],
        max_error_x = [maximum(abs.(tsit5_data.error_x)), maximum(abs.(euler_data.error_x))],
        max_error_y = [maximum(abs.(tsit5_data.error_y)), maximum(abs.(euler_data.error_y))],
        max_error_z = [maximum(abs.(tsit5_data.error_z)), maximum(abs.(euler_data.error_z))]
    )
    
    CSV.write("error_statistics.csv", error_stats)
    println("\nError Statistics:")
    println(error_stats)
else
    if isnothing(tsit5_data)
        println("\nWarning: Tsit5 data not available")
    end
    if isnothing(euler_data)
        println("\nWarning: Euler data not available")
    end
end

# Print file existence check
files_to_check = [
    "tsit5_data.csv",
    "euler_data.csv",
    "tsit5_timing.csv",
    "euler_timing.csv"
]

println("\nFile Status:")
for file in files_to_check
    exists = isfile(file)
    if exists
        file_size = Base.filesize(file)  # Using Base.filesize instead of filesize
        println("$file: Found ($file_size bytes)")
    else
        println("$file: Missing")
    end
end