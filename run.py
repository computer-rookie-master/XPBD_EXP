import subprocess

# build

filePath = "build"
runCommand = ""
if filePath:
    print("delete, build and run")
    runCommand = ('rm -rf\nbuild\nmkdir build\ncd build\ncmake ..\nmake\n./XPBD_EXP')
else:
    print("build and run")
    runCommand = ('mkdir build\ncd build\ncmake ..\nmake\n./XPBD_EXP')
subprocess.call([runCommand], shell = True)