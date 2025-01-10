
add_compile_options(-march=rv32imc_zicsr_zifencei)
add_link_options(-march=rv32imc_zicsr_zifencei)

set(CMAKE_C_FLAGS
    "-march=rv32imc_zicsr_zifencei"
    CACHE STRING "C Compiler Base Flags")
set(CMAKE_CXX_FLAGS
    "-march=rv32imc_zicsr_zifencei"
    CACHE STRING "C++ Compiler Base Flags")
set(CMAKE_EXE_LINKER_FLAGS
    "-nostartfiles -march=rv32imc_zicsr_zifencei --specs=nosys.specs"
    CACHE STRING "Linker Base Flags")
