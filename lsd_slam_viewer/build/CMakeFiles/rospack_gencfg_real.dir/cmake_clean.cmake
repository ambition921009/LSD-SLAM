file(REMOVE_RECURSE
  "../msg_gen"
  "../msg_gen"
  "../src/lsd_slam_viewer/msg"
)

# Per-language clean rules from dependency scanning.
foreach(lang )
  include(CMakeFiles/rospack_gencfg_real.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
