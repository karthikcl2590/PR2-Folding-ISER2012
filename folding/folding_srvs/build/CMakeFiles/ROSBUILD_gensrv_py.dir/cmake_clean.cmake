FILE(REMOVE_RECURSE
  "../srv_gen"
  "../src/folding_srvs/srv"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_py"
  "../src/folding_srvs/srv/__init__.py"
  "../src/folding_srvs/srv/_GetTable.py"
  "../src/folding_srvs/srv/_ExecuteFold.py"
  "../src/folding_srvs/srv/_GoToGrip.py"
  "../src/folding_srvs/srv/_AdjustFold.py"
  "../src/folding_srvs/srv/_FoldingStance.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
