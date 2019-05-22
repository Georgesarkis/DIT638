# Controller Microsercive

This is a microservice which acts as an intermediary between the other microservices of this project and other existing microservices on the car.

You can build the software module for `amd64` with this command:
```bash
docker build -t <file-name> -f Dockerfile.amd64 .
```
To build for armhf platform, simply replace the Dockerfile file extension with "armhf".

You can run the software module:
```bash
docker run --rm -ti --init --net=host --ipc=host -v /tmp:/t <file-name> --cid=112 
```

In order to communicate with the other microservice, ensure that they have the same CID.