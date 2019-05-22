# V2X Microservice

This microservice will run a program which may send a direction instruction.
In return, the program receives a confirmation or rejection of the instruction.

It takes one commandline argument "CID" which represents the channel ID where it is sending and receiving.

You can build the software module for `amd64` with this command:
```bash
docker build -t <file-name> -f Dockerfile.amd64 .
```
To build for armhf platform, simply replace the Dockerfile file extension with "armhf".

You can run the software module: for `armhf` or `amd64` as follows:
```bash
docker run --rm -ti --init --net=host --ipc=host -v /tmp:/tmp send_v2.armhf --cid=112
```

