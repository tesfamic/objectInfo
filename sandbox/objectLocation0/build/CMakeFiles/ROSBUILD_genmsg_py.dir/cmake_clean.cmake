FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/objectLocation0/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/objectLocation0/msg/__init__.py"
  "../src/objectLocation0/msg/_coordinateContent.py"
  "../src/objectLocation0/msg/_coordinateArray.py"
  "../src/objectLocation0/msg/_objectPos.py"
  "../src/objectLocation0/msg/_objectPosArray.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
