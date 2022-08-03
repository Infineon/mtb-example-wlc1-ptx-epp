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

echo "Modifying Makefile to enable QC_AFC build"
echo "---------------------------"

REM Backup the config.h
cp Makefile Makefile_bkup

REM Use PD override
sed -i 's/BATTERY_CHARGING_ENABLE=0/BATTERY_CHARGING_ENABLE=1/' Makefile
sed -i 's/QC_AFC_SNK_EN=0/QC_AFC_SNK_EN=1/' Makefile

echo "Building the Application"
echo "---------------------------"

%CY_TOOLS_PATH%/modus-shell/bin/bash.exe -l -c "cd %basedir%; make build" || exit 1

echo "Build completed"
echo "---------------------------"

REM Restore the config.h
mv Makefile_bkup Makefile

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

set "basedir=%MTB_FW_BASE%"

set "mtb_shared=%basedir%\..\mtb_shared"
set "build=%basedir%\build\WLC1115-68LQXQ\Release"
set "product_name=ccgx"
set "ce_name=mtb-example-wlc1_ptx_epp"
set "cp_name=%ce_name%_qc_afc"
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

REM Retreive commit ID --------------------------------------------------------
git rev-parse "refs/remotes/origin/%git_branch%^{commit}" > tmp.txta
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

echo "%date% %time%: %cp_name% binaries copy started."

if exist "%destbase%\%cp_name%" (
    DEL /Q "%destbase%\%cp_name%"
    )    
mkdir "%destbase%\%cp_name%"

REM Copy the standard FW hex file ---------------------------------------------
xcopy "%build%\binaries\%ce_name%.hex" "%destbase%\%cp_name%" /Q /Y
ren "%destbase%\%cp_name%\%ce_name%.hex" "%cp_name%_%verstr%.hex"

REM Copy the cyacd2 file ---------------------------------------------
xcopy "%build%\binaries\%ce_name%.cyacd2" "%destbase%\%cp_name%" /Q /Y
ren "%destbase%\%cp_name%\%ce_name%.cyacd2" "%cp_name%_%verstr%.cyacd2"

REM Copy the PP FW hex file ---------------------------------------------------
xcopy "%build%\binaries\%ce_name%_pp.hex" "%destbase%\%cp_name%" /Q /Y
ren "%destbase%\%cp_name%\%ce_name%_pp.hex" "%cp_name%_pp_%verstr%.hex"

REM Zip up the binary files ---------------------------------------------------
"%archive_tool%" a "%destbase%\%cp_name%_%fw_major%_%fw_minor%_%fw_patch%_%fw_build%_binaries.zip" "%destbase%\%cp_name%"
    
echo "%date% %time%: Packaging finished."
