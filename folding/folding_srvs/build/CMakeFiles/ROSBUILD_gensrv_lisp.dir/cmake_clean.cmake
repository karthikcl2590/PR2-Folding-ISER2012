FILE(REMOVE_RECURSE
  "../srv_gen"
  "../src/folding_srvs/srv"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_lisp"
  "../srv_gen/lisp/GetTable.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_GetTable.lisp"
  "../srv_gen/lisp/ExecuteFold.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_ExecuteFold.lisp"
  "../srv_gen/lisp/GoToGrip.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_GoToGrip.lisp"
  "../srv_gen/lisp/AdjustFold.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_AdjustFold.lisp"
  "../srv_gen/lisp/FoldingStance.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_FoldingStance.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
