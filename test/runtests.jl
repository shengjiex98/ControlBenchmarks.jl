using ControlBenchmarks
using ControlSystemsBase
using Test

@testset "ControlBenchmarks.jl" begin
    @test c2d(benchmarks[:F1], 0.02).A ≈ [1.0 0.12999999999999998; 0.0 1.0]
    @test delay_lqr(benchmarks[:F1], 0.02) ≈ [0.5829779235411586 0.9271753376618778 0.3501109341069689]
    @test pole_place(benchmarks[:F1], 0.02) ≈ [0.19538461538461566 0.5207 0.20000000000000026]
end
