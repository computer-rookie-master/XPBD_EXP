import subprocess

# build

filePath = "build"
runCommand = ""
if filePath:
    print("delete, build and run")
    runCommand = ('rm -rf\nbuild\nmkdir build\ncd build\ncmake -DCMAKE_BUILD_TYPE=Debug ..\nmake')
else:
    print("build and run")
    runCommand = ('mkdir build\ncd build\ncmake -DCMAKE_BUILD_TYPE=Debug ..\nmake')
subprocess.call([runCommand], shell = True)