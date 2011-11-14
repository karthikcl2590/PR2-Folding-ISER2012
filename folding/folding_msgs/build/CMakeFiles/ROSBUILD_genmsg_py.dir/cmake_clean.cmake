FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/folding_msgs/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/folding_msgs/msg/__init__.py"
  "../src/folding_msgs/msg/_Line2D.py"
  "../src/folding_msgs/msg/_PolyStamped.py"
  "../src/folding_msgs/msg/_GripTarget.py"
  "../src/folding_msgs/msg/_FoldTraj.py"
  "../src/folding_msgs/msg/_Point2D.py"
  "../src/folding_msgs/msg/_GripsTarget.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
