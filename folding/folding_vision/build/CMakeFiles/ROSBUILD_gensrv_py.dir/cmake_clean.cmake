FILE(REMOVE_RECURSE
  "../srv_gen"
  "../src/folding_vision/srv"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_py"
  "../src/folding_vision/srv/__init__.py"
  "../src/folding_vision/srv/_LocatePolygon.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
