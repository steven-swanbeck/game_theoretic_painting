file(REMOVE_RECURSE
  "libpoint.a"
  "libpoint.pdb"
)

# Per-language clean rules from dependency scanning.
foreach(lang )
  include(CMakeFiles/point.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
