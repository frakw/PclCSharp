break>release_lib_list.txt
setlocal enabledelayedexpansion
set "parentfolder=%CD%"
for /r ./boost %%g in (*mt-1_64.lib) do (
  set "var=%%g"
  set var=!var:%parentfolder%=!
  set var=!var!
  echo !var! >> release_lib_list.txt
)
for /r ./pcl-1.8 %%g in (*release.lib) do (
  set "var=%%g"
  set var=!var:%parentfolder%=!
  set var=!var!
  echo !var! >> release_lib_list.txt
)
for /r ./Qhull %%g in (*_release.lib) do (
  set "var=%%g"
  set var=!var:%parentfolder%=!
  echo !var! >> release_lib_list.txt
)
for /r ./VTK %%g in (*8.0.lib) do (
  set "var=%%g"
  set var=!var:%parentfolder%=!
  echo !var! >> release_lib_list.txt
)
echo \flann\flann.lib >> release_lib_list.txt
echo \flann\flann_cpp.lib >> release_lib_list.txt
echo \OpenNI2\OpenNI2.lib >> release_lib_list.txt