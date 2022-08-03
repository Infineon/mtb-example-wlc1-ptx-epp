@echo off
REM Usage model ---------------------------------------------------------------
REM The script updates the build number in the version file. The script needs
REM to be copied to the build machine and executed locally. The script is 
REM expected to be triggered from the Jenkins build server during every build.
REM
REM package_prebuild.bat <branch> <repo URL>
REM
REM <branch> is the branch for the build
REM <repo URL> is the URL to the repo
REM ---------------------------------------------------------------------------

REM Set environment variables -------------------------------------------------

set argc=0
for %%x in (%*) do set /A argc+=1

if %argc% lss 2 (
    echo "ERROR: Invalid arguments"
   exit 1
)

REM echo must not be turned ON during the user and password information load
@echo off
REM GIT login data. This should not be checked into source control and should
REM be available only on build machine --------------------------------------
set "GIT_USER="
set "GIT_PASS="

if "%GIT_USER%"=="" (
    echo "Pre-build environment not ready. Script is not complete."
    exit 1
)

if "%GIT_PASS%"=="" (
    echo "Pre-build environment not ready. Script is not complete."
    exit 1
)

REM The base directory needs to be updated to the actual location of workspace
REM in the machine. The current location is as per the build machine. ---------
set "basedir=C:\MTB_TEST\workspace"

REM Identify and fetch the repository -----------------------------------------
set GIT_BRANCH=%1
echo %2| sed s/\/\//\/\/%GIT_USER%:%GIT_PASS%@/ >tmp.txt
set /p GIT_URL=<tmp.txt

echo %2| sed 's/^.*\(.*\)\//\1/' | sed 's/\..*//' >tmp.txt
set /p GIT_REPO_NAME=<tmp.txt

del tmp.txt

echo Updating build inforamation for %GIT_REPO_NAME% branch %GIT_BRANCH%

mkdir "%basedir%\%GIT_REPO_NAME%"
cd "%basedir%\%GIT_REPO_NAME%"

set GIT_REPO_PREP=0
git rev-parse --is-inside-work-tree || set GIT_REPO_PREP=1

if %GIT_REPO_PREP% == 1 (
    cd ..
    rm -rf %GIT_REPO_NAME%
    git clone --branch %GIT_BRANCH% --no-tags --depth 1 %GIT_URL%
    cd %GIT_REPO_NAME%
)

git rev-parse --is-inside-work-tree
if not %ERRORLEVEL% == 0 (
    echo "Pre-build environment not ready. Exiting ..."
    exit %ERRORLEVEL%
)

git fetch --no-tags --force %GIT_URL% +refs/heads/%GIT_BRANCH%*:refs/remotes/origin/%GIT_BRANCH%*
git checkout %GIT_BRANCH%
git reset --hard
git pull

set "version_file=%basedir%\%GIT_REPO_NAME%\ccgx_version.h"
set "build_num_var=FW_BUILD_NUMBER"

REM Replace build numbers -----------------------------------------------------
sed -i "s/#define %build_num_var%.*/#define %build_num_var%                                 (%BUILD_NUMBER%)/" %version_file%

REM Submit build number updates -----------------------------------------------
git add "ccgx_version.h"
git commit -am "Build number incremented to %BUILD_NUMBER%"
git push

echo "Build number incremented to %BUILD_NUMBER%"
