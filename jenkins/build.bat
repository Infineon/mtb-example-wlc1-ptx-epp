@echo off

echo "Extracting path"

echo %WORKSPACE% > tmp1.txt
sed -r "s/\\\/\//g" tmp1.txt > tmp.txt
set /p basedir=<tmp.txt
del tmp1.txt
del tmp.txt

echo "Exporting Remote Manifest"
echo "---------------------------"

set "CyRemoteManifestOverride=https://git-ore.aus.cypress.com/VenkatK/mtb-wlc1-src-manifest/-/raw/main/mtb-wlc1-super-manifest.xml"
set "CY_SUPPORTED_KITS=CCG7S,CCG7D"
set "CY_TOOLS_PATH=C:/Users/fwbuild/ModusToolbox/tools_2.4/"

REM First ensure that output binaries are deleted to avoid stale copy
rmdir "%WORKSPACE%\build\WLC1115-68LQXQ\Release\binaries" /s /q

echo "Getting Application and Middlewares"
echo "---------------------------"

%CY_TOOLS_PATH%/modus-shell/bin/bash.exe -l -c "cd %basedir%; make getlibs" || exit 1

echo "Building the Application"
echo "---------------------------"

%CY_TOOLS_PATH%/modus-shell/bin/bash.exe -l -c "cd %basedir%; make build" || exit 1

echo "Build completed"
echo "---------------------------"

REM Now do post build for copying the artifacts
echo "Starting build archive"
echo "---------------------------"

REM Identify the workspace location for firmware. If the location is available
REM from Jenkins use it. Otherwise use it directly from the system variable ---
if not "%WORKSPACE%"=="" (
    set "MTB_FW_BASE=%WORKSPACE%"
)

if  "%MTB_FW_BASE%"=="" (
    echo "ERROR: Firmware source directory not found"
    exit 1
)

echo "Starting build archive"
echo "---------------------------"

set "basedir=%MTB_FW_BASE%"

set "mtb_shared=%basedir%\..\mtb_shared"
set "build=%basedir%\build\WLC1115-68LQXQ\Release"
set "product_name=ccgx"
set "ce_name=mtb-example-wlc1_ptx_epp"
set "app_name=wi"
set "version_name=%product_name%_version.h"
set "version_file=%basedir%\%version_name%"
set "app_version_file=%basedir%\app_version.h"
set "major_version_var=FW_MAJOR_VERSION"
set "minor_version_var=FW_MINOR_VERSION"
set "patch_version_var=FW_PATCH_VERSION"
set "build_num_var=FW_BUILD_NUMBER"
set "app_major_version_var=APP_MAJOR_VERSION"
set "app_minor_version_var=APP_MINOR_VERSION"
set "app_ext_cir_num_var=APP_EXT_CIR_NUM"

REM Setup paths and tools -----------------------------------------------------
set "destbase=%basedir%\archive"
set "archive_tool=C:\Program Files\7-Zip\7z"

REM Make projects writable and remove any previous stale build outputs --------
rmdir "%destbase%" /s /q
mkdir "%destbase%"

REM Retreive commit ID --------------------------------------------------------
git rev-parse "refs/remotes/%git_branch%^{commit}" > tmp.txta
set /p GIT_COMMIT=<tmp.txt
echo "GIT last commit: %GIT_COMMIT%"

REM Retrieve the base version for the stack. ----------------------------------
grep -E "%major_version_var%\s*\(" "%version_file%" > tmp1.txt
sed -r "s/.*\(([0-9]*).*/\1/g" tmp1.txt >tmp.txt
set /p fw_major=<tmp.txt
grep -E "%minor_version_var%\s*\(" "%version_file%" > tmp1.txt
sed -r "s/.*\(([0-9]*).*/\1/g" tmp1.txt >tmp.txt
set /p fw_minor=<tmp.txt
grep -E "%patch_version_var%\s*\(" "%version_file%" > tmp1.txt
sed -r "s/.*\(([0-9]*).*/\1/g" tmp1.txt >tmp.txt
set /p fw_patch=<tmp.txt
grep -E "%build_num_var%\s*\(" "%version_file%" > tmp1.txt
sed -r "s/.*\(([0-9]*).*/\1/g" tmp1.txt >tmp.txt
set /p fw_build=<tmp.txt

grep -E "%app_major_version_var%\s*\(" "%app_version_file%" > tmp1.txt
sed -r "s/.*\(([0-9,x,X,a-f,A-F]*).*/\1/g" tmp1.txt >tmp.txt
set /p app_major_str=<tmp.txt
set /a app_major=%app_major_str%
grep -E "%app_minor_version_var%\s*\(" "%app_version_file%" > tmp1.txt
sed -r "s/.*\(([0-9,x,X,a-f,A-F]*).*/\1/g" tmp1.txt >tmp.txt
set /p app_minor_str=<tmp.txt
set /a app_minor=%app_minor_str%
grep -E "%app_ext_cir_num_var%\s*\(" "%app_version_file%" > tmp1.txt
sed -r "s/.*\(([0-9,x,X,a-f,A-F]*).*/\1/g" tmp1.txt >tmp.txt
set /p ckt_num_str=<tmp.txt
set /a ckt_num=%ckt_num_str%

set verstr=%fw_major%_%fw_minor%_%fw_patch%_%fw_build%_%app_major%_%app_minor%_%ckt_num%_%app_name%

del tmp1.txt
del tmp.txt

echo "%date% %time%: %ce_name% binaries copy started."

if exist "%destbase%\%ce_name%" (
    DEL /Q "%destbase%\%ce_name%"
    )    
mkdir "%destbase%\%ce_name%"
mkdir "%destbase%\%ce_name%\Repo_Assests"

REM Copy the bootloader file and rename it ------------------------------------
xcopy "%basedir%\Bootloader\*.hex" "%destbase%\%ce_name%"

REM Copy the standard FW hex file ---------------------------------------------
xcopy "%build%\binaries\%ce_name%.hex" "%destbase%\%ce_name%" /Q /Y
ren "%destbase%\%ce_name%\%ce_name%.hex" "%ce_name%_%verstr%.hex"

REM Copy the cyacd2 file ---------------------------------------------
xcopy "%build%\binaries\%ce_name%.cyacd2" "%destbase%\%ce_name%" /Q /Y
ren "%destbase%\%ce_name%\%ce_name%.cyacd2" "%ce_name%_%verstr%.cyacd2"

REM Copy the PP FW hex file ---------------------------------------------------
xcopy "%build%\binaries\%ce_name%_pp.hex" "%destbase%\%ce_name%" /Q /Y
ren "%destbase%\%ce_name%\%ce_name%_pp.hex" "%ce_name%_pp_%verstr%.hex"

REM Copy the version.xml files from all middlewares ---------------------------
xcopy "%mtb_shared%\ccgxAppCommon\feature-wicg\version.xml" "%destbase%\%ce_name%\Repo_Assests" /Q /Y
ren "%destbase%\%ce_name%\Repo_Assests\version.xml" "ccgxAppCommon_%verstr%.xml"

xcopy "%mtb_shared%\core-lib\develop\version.xml" "%destbase%\%ce_name%\Repo_Assests" /Q /Y
ren "%destbase%\%ce_name%\Repo_Assests\version.xml" "core-lib_%verstr%.xml"

xcopy "%mtb_shared%\core-make\latest-v1.X\version.xml" "%destbase%\%ce_name%\Repo_Assests" /Q /Y
ren "%destbase%\%ce_name%\Repo_Assests\version.xml" "core-make_%verstr%.xml"

xcopy "%mtb_shared%\mtb-pdl-cat2\feature-wicg-1.6\version.xml" "%destbase%\%ce_name%\Repo_Assests" /Q /Y
ren "%destbase%\%ce_name%\Repo_Assests\version.xml" "mtb-pdl-cat2_%verstr%.xml"

xcopy "%mtb_shared%\pdstack\feature-wicg\version.xml" "%destbase%\%ce_name%\Repo_Assests" /Q /Y
ren "%destbase%\%ce_name%\Repo_Assests\version.xml" "pdstack_%verstr%.xml"

xcopy "%mtb_shared%\qistack\develop\version.xml" "%destbase%\%ce_name%\Repo_Assests" /Q /Y
ren "%destbase%\%ce_name%\Repo_Assests\version.xml" "qistack_%verstr%.xml"

xcopy "%mtb_shared%\recipe-make-cat2\latest-v1.X\version.xml" "%destbase%\%ce_name%\Repo_Assests" /Q /Y
ren "%destbase%\%ce_name%\Repo_Assests\version.xml" "recipe-make-cat2_%verstr%.xml"

xcopy "%mtb_shared%\TARGET_WLC1\main\version.xml" "%destbase%\%ce_name%\Repo_Assests" /Q /Y
ren "%destbase%\%ce_name%\Repo_Assests\version.xml" "TARGET_WLC1_%verstr%.xml"

xcopy "%mtb_shared%\wlchpi\develop\version.xml" "%destbase%\%ce_name%\Repo_Assests" /Q /Y
ren "%destbase%\%ce_name%\Repo_Assests\version.xml" "wlchpi_%verstr%.xml"

REM Copy the default xml file ---------------------------------------------
xcopy "%basedir%\config.xml" "%destbase%\%ce_name%" /Q /Y
ren "%destbase%\%ce_name%\config.xml" "%ce_name%_config_%verstr%.xml"

REM Zip up the binary files ---------------------------------------------------
"%archive_tool%" a "%destbase%\%ce_name%_%fw_major%_%fw_minor%_%fw_patch%_%fw_build%_binaries.zip" "%destbase%\%ce_name%"
    
echo "%date% %time%: Packaging finished."
