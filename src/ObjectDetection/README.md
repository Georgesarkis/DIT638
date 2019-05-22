# Object detection micro service

This is a micro service used for different kinds of detection of objects, it uses color and shape detection as well as ultrasonic sensor and IR sensor detection. 

Commandline arguments needed to run this microservice:

You can build the software module for `amd64` with this command:
```bash
docker build -t <file-name> -f Dockerfile.amd64 .
```
To build for armhf platform, simply replace the Dockerfile file extension with "armhf".

You can run the software module:
```bash
docker run --rm -ti --init --net=host --ipc=host -v /tmp:/tmp <file-name> --cid=112 --name=img.argb --width=640 --height=480
```
Optional commandline arguments are "--verbose" and "--video". 
* If you include verbose, it will print out interesting stuff.
* If you include video, it will show the video frames of the video input.