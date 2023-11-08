break>debug_lib_list.txt
setlocal enabledelayedexpansion
set "parentfolder=%CD%"
for /r ./boost %%g in (*mt-gd-1_64.lib) do (
  set "var=%%g"
  set var=!var:%parentfolder%=!
  echo !var! >> debug_lib_list.txt
)
for /r ./flann %%g in (*gd.lib) do (
  set "var=%%g"
  set var=!var:%parentfolder%=!
  echo !var! >> debug_lib_list.txt
)
for /r ./pcl-1.8 %%g in (*debug.lib) do (
  set "var=%%g"
  set var=!var:%parentfolder%=!
  echo !var! >> debug_lib_list.txt
)
for /r ./Qhull %%g in (*_d.lib) do (
  set "var=%%g"
  set var=!var:%parentfolder%=!
  echo !var! >> debug_lib_list.txt
)
for /r ./VTK %%g in (*gd.lib) do (
  set "var=%%g"
  set var=!var:%parentfolder%=!
  echo !var! >> debug_lib_list.txt
)
echo \OpenNI2\OpenNI2.lib >> debug_lib_list.txt