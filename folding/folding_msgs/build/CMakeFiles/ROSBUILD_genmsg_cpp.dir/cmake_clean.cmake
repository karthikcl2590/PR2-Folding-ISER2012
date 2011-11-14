FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/folding_msgs/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "../msg_gen/cpp/include/folding_msgs/Line2D.h"
  "../msg_gen/cpp/include/folding_msgs/PolyStamped.h"
  "../msg_gen/cpp/include/folding_msgs/GripTarget.h"
  "../msg_gen/cpp/include/folding_msgs/FoldTraj.h"
  "../msg_gen/cpp/include/folding_msgs/Point2D.h"
  "../msg_gen/cpp/include/folding_msgs/GripsTarget.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
