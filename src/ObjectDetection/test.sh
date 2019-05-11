clear

echo "----------- Testing for Complexity -----------"


# Loops thru the source files that have a extention of .cpp and aplied the formatting command to them
for f in src/*.cpp; do
  pmccabe $f
done

# Does the same thing as cpp files , but ingnores the large libriary files
for f in src/*.hpp; do
  pmccabe $f
done

echo "----------- Test for Complexity Complete -----------"
echo ""


echo "----------- Testing for Errors -----------"

# Loops thru the source files that have a extention of .cpp and aplied the formatting command to them
for f in src/*.cpp; do
  cppcheck -q $f
done

# Does the same thing as cpp files , but ingnores the large libriary files
for f in src/*.hpp; do
  cppcheck -q $f
done

echo "----------- Test for Errors Complete -----------"
echo ""

# Get the curent path
path=$(pwd)

# The cmake commands
cmake -H$path/src -B$path/bin
cmake --build bin

echo "----------- Testing for Memory leaks -----------"

valgrind docker run --rm -ti --init --ipc=host -v /tmp:/tmp -e DISPLAY=$DISPLAY object_detection:0.0.1 --name=video0.argb --cid=112 --width=640 --height=480 --verbose --video --nrsign=3 --minarea=1000 --carspeed=0.11

echo "----------- Test for Memory leaks Complete -----------"
