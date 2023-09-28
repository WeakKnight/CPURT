import platform
import os

def main():     
    systemName = platform.system()
    print("Building On " + systemName + " System")

    dir_path = os.path.dirname(os.path.realpath(__file__))
    print("Current Path is " + dir_path)

    if not os.path.isdir(dir_path + '/Build/'):
        os.mkdir(dir_path + '/Build/')

    if not os.path.isdir(dir_path + '/Bin/'):
        os.mkdir(dir_path + '/Bin/')

    os.chdir(dir_path + "/Build")
    if systemName == "Windows":
        os.system('cmake -G "Visual Studio 17 2022" -A x64 ../')
    elif systemName == "Darwin":
        os.system('cmake -G "Xcode" ../')

if __name__ == '__main__':
    main()