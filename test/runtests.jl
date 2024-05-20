using ControlBenchmarks
using Test

@testset "ControlBenchmarks.jl" begin
    sys = benchmarks[:F1T]
    K = delay_lqr(sys, 0.02)

    @test K ≈ [0.58 0.93 0.35] atol=0.01
end
