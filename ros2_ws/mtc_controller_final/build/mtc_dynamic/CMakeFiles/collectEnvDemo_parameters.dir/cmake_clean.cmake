file(REMOVE_RECURSE
  "include/collectEnvDemo_parameters.hpp"
  "include/mtc_dynamic/collectEnvDemo_parameters.hpp"
)

# Per-language clean rules from dependency scanning.
foreach(lang )
  include(CMakeFiles/collectEnvDemo_parameters.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
