chip=$1

dirOrg=$(pwd)
scriptDIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"


cd $scriptDIR/../Libraries/PeriphDrivers/Documentation

mkdir ./$chip
sed -i 's/<__DEVICE_NAME__>/'$chip'/' ./Doxyfile
doxygen ./Doxyfile

cd $dirOrg
