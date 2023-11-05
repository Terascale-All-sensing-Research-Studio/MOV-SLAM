# MoV-SLAM
Motion Vector SLAM pipeline

v0.1 November 5th 2022
Authors - 

We present MoV-SLAM, or Motion Vectors Simultaneous Localization and Mapping, an approach for real-time visual SLAM (vSLAM) using low-resource single-CPU lightweight computing devices on unmanned aerial vehicles (UAVs) and microaerial vehicles (MAVs).

We provide examples to run MoV-SLAM for both the TartanAir dataset and EuRoC dataset. 

# 1.0 License

MoV-SLAM takes its roots from ORB-SLAM3, which is licensed under [GPLv3](https://www.gnu.org/licenses/gpl-3.0.en.html). Therefore, MoV-SLAM is also licensed under GPLv3 and should only be used for academic/research interests. 

# 2.0 Building

MoV-SLAM is built and ran within a docker container. Provided in the root directory of the project is the Dockerfile. MoV-SLAM has been successfully ran within Docker on a Macbook M1 and Raspberry Pi.

The only requirements on the host machine is [XQuartz](https://www.xquartz.org/) for the Mac, to run the X display, and [Docker](https://www.docker.com). Docker renders to the X11 display through forwarding to the host. From XQuartz, you need to enable network connections, see [here](https://gist.github.com/cschiewek/246a244ba23da8b9f0e7b11a68bf3285).

To build the docker images you need to run the following command from the root directory of the project:

```
docker build -t movslam:latest .
```

This process may take 1-2 hours depending on the machine. Once completed, you can run and enter the container with the following command:

```
docker run --env="DISPLAY=host.docker.internal:0" -i -t movslam:latest
```

The display environment variable will pass the host docker internal display for forwarding from the container. 

MoV-SLAM will be built in the /home/MOV_SLAM folder. Within this folder is the build folder, containing the binaries for both the mono and stereo setup.
If at any point you want to rebuild MoV-SLAM, you can either run make from within the build folder or execute the build.sh script in the root MOV_SLAM folder.

To run MoV-SLAM you will use the following command:

```
./mono_video_tartan /home/MOV_SLAM/Examples/Monocular/TartanAir.yaml tcp://IPADDRESS:8000
```

You will replace IPADDRESS with the remote video encoder. For FFMPEG remote video encoding on another machine, you will need to install FFMPEG and the image dataset. For TartanAir, the remote execution command would be the following:

```
ffmpeg -framerate 30 -pattern_type glob -i "PATH_TARTANAIR/oldtown/Hard/P003/image_left/*left.png" -c:v libx264 -preset "fast" -tune "film" -x264opts "partitions=p8x8,p4x4,i8x8:keyint=1000:me=umh:merange=64:subme=6:bframes=0:ref=1" -f mpegts tcp://IPADDRESS:8000/?listen
```

Let's breakdown this command -

ffmpeg : Locally installed FFMPEG instance. Recommend version 4.0 or later. The version used for the paper is 5.1.

-framerate : This is the recommended video transcoded framerate. This should match the recorded image framerate from the dataset. For example, 20 should be used for EuRoC.

-pattern_type glob : This indicates to FFMPEG it should read in the input frames and transcode them using the specified video encoder (libx264)

-i "PATH_TARTANAIR/oldtown/Hard/P003/image_left/*left.png" : This is the input folder with the dataset images. You need to wildcard the images as shown here depending on the dataset.

-c:v libx264 : This information FFMPEG we will be using the libx264 encoder for the transcoding.

-preset "fast" -tune "film : Libx264 has various presets and tuning parameters you can use depending on the quality and speed you are looking for. For most dataset images we have found a preset of fast, which is fast transcoding with minimal degradation of quality, with film tuning to be optimal.

-x264opts "partitions=p8x8,p4x4,i8x8:keyint=1000:me=umh:merange=64:subme=6:bframes=0:ref=4" : partitions is the recommended macroblock partition size. We use all P/I frame partition sizes available. A higher keyint will increase the number of P frames, and therefore keep longer tracks. UMH is the recommended search motion vector algorithm. We have found 64 pixels to be optimal balance between performance and quality for mv searching. Lastly, turn off bframes and set the ref to your ideal number. In this case, we use a P reference of 4. Lower reference will increase performance but shorten tracks.

-f mpegts tcp://IPADDRESS:8000/?listen : This tells FFMPEG to listen on the specified port for an incoming connection. The IPADDRESS should be replaced with the network IP address on the video encoding machine. 


For stereo, you need to have both the left and right images in the same folder you will be using as the input into the encoder. The command for ffmpeg needs to include "frame-packing=5" to the x264opts. This tells libx264 and FFMPEG we will be using interlacing frame packing for 3D stereo. Also, the reference number needs to be in multiple of 2, with a ref no smaller than 2. 
