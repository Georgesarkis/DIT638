clear

echo "Formatting Files ... "

# Loops thru the source files that have a extention of .cpp and aplied the formatting command to them
for f in src/*.cpp; do
  echo "Formatting $f ..."
  clang-format -i $f
done

# Does the same thing as cpp files , but ingnores the large libriary files
for f in src/*.hpp; do
  echo "Formatting $f ..."
  clang-format -i $f
done

echo "Done Formatting ... "

docker build -t controller:0.0.1 -f Dockerfile.amd64 .
