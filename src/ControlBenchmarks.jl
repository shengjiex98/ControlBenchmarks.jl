module ControlBenchmarks

export benchmarks
include("models.jl")

export delay_lqr, pole_place, augment
include("controllers.jl")

end
