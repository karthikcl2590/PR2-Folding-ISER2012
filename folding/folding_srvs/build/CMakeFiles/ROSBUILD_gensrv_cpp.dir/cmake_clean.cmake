FILE(REMOVE_RECURSE
  "../srv_gen"
  "../src/folding_srvs/srv"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_cpp"
  "../srv_gen/cpp/include/folding_srvs/GetTable.h"
  "../srv_gen/cpp/include/folding_srvs/ExecuteFold.h"
  "../srv_gen/cpp/include/folding_srvs/GoToGrip.h"
  "../srv_gen/cpp/include/folding_srvs/AdjustFold.h"
  "../srv_gen/cpp/include/folding_srvs/FoldingStance.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
