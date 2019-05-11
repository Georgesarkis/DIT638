echo ----------- Program Starting -----------

start=$(date +%s%N)

docker run --rm -ti --init --ipc=host -v /tmp:/tmp -e DISPLAY=$DISPLAY object_detection:0.0.1 --name=video0.argb --cid=112 --width=640 --height=480 --verbose --video --nrsign=3 --minarea=1000 --carspeed=0.11

dur=$(echo "$(date +%s%N) - $start" | bc)

echo ----------- Program Finsihed -----------

printf "Execution took : %d nanoseconds \n" $dur
