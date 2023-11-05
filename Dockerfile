FROM ubuntu:22.04

ENV TZ=America/New_York
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

WORKDIR "/home"

RUN apt-get update && apt-get upgrade -y

# Install dependencies
RUN apt-get update && apt-get upgrade -y  &&  apt-get install -y \
	git sudo cmake g++ libglu1-mesa-dev freeglut3-dev mesa-common-dev libeigen3-dev \
	libglew-dev libboost-all-dev libssl-dev libopenblas-dev

# Install FFMPEG
RUN apt-get install -y wget unzip nasm libass-dev vorbis-tools libtheora-dev

WORKDIR "/home"
RUN wget https://downloads.sourceforge.net/opencore-amr/fdk-aac-2.0.2.tar.gz
RUN tar -xf fdk-aac-2.0.2.tar.gz
WORKDIR "/home/fdk-aac-2.0.2"
RUN ./configure --prefix=/usr --disable-static
RUN make
RUN make install

WORKDIR "/home"
RUN wget https://downloads.sourceforge.net/freetype/freetype-2.12.1.tar.xz
RUN tar -xf freetype-2.12.1.tar.xz
WORKDIR "/home/freetype-2.12.1"
RUN sed -ri "s:.*(AUX_MODULES.*valid):\1:" modules.cfg
RUN sed -r "s:.*(#.*SUBPIXEL_RENDERING) .*:\1:" \
    -i include/freetype/config/ftoption.h
RUN ./configure --prefix=/usr --enable-freetype-config --disable-static
RUN make
RUN make install

WORKDIR "/home"
RUN wget https://downloads.sourceforge.net/lame/lame-3.100.tar.gz
RUN tar -xf lame-3.100.tar.gz
WORKDIR "/home/lame-3.100"
RUN ./configure --prefix=/usr --enable-mp3rtp --disable-static
RUN make
RUN make pkghtmldir=/usr/share/doc/lame-3.100 install

WORKDIR "/home"
RUN wget https://archive.mozilla.org/pub/opus/opus-1.3.1.tar.gz
RUN tar -xf opus-1.3.1.tar.gz
WORKDIR "/home/opus-1.3.1"
RUN ./configure --prefix=/usr    \
            --disable-static \
            --docdir=/usr/share/doc/opus-1.3.1
RUN make
RUN make install

WORKDIR "/home"
RUN wget https://downloads.xiph.org/releases/vorbis/libvorbis-1.3.7.tar.xz
RUN tar -xf libvorbis-1.3.7.tar.xz
WORKDIR "/home/libvorbis-1.3.7"
RUN ./configure --prefix=/usr --disable-static
RUN make
RUN make install
RUN install -v -m644 doc/Vorbis* /usr/share/doc/libvorbis-1.3.7

WORKDIR "/home"
RUN wget https://anduin.linuxfromscratch.org/BLFS/x264/x264-20220819.tar.xz
RUN tar -xf x264-20220819.tar.xz
WORKDIR "/home/x264-20220819"
RUN ./configure --prefix=/usr --enable-shared --disable-cli 
RUN make
RUN make install

WORKDIR "/home"
RUN wget https://github.com/webmproject/libvpx/archive/v1.12.0/libvpx-1.12.0.tar.gz
RUN tar -xf libvpx-1.12.0.tar.gz
WORKDIR "/home/libvpx-1.12.0"
RUN sed -i 's/cp -p/cp/' build/make/Makefile
RUN mkdir libvpx-build
RUN cd libvpx-build
RUN ./configure --prefix=/usr    \
             --enable-shared  \
             --disable-static
RUN make
RUN make install

WORKDIR "/home"
RUN wget https://anduin.linuxfromscratch.org/BLFS/x265/x265-20220819.tar.xz
RUN tar -xf x265-20220819.tar.xz
WORKDIR "/home/x265-20220819"
RUN mkdir bld
WORKDIR "/home/x265-20220819/bld"
RUN cmake -DCMAKE_INSTALL_PREFIX=/usr \
      -DGIT_ARCHETYPE=1 ../source
RUN make
RUN make install
RUN rm -vf /usr/lib/libx265.a 

WORKDIR "/home"
RUN wget https://ffmpeg.org/releases/ffmpeg-4.4.3.tar.xz
RUN tar -xf ffmpeg-4.4.3.tar.xz
COPY ffmpeg-ref-patch.patch /home/
WORKDIR "/home/ffmpeg-4.4.3"
RUN patch -p0 < /home/ffmpeg-ref-patch.patch
RUN ./configure --prefix=/usr        \
            --enable-gpl         \
            --enable-version3    \
            --enable-nonfree     \
            --disable-static     \
            --enable-shared      \
            --disable-debug      \
            --enable-libass      \
            --enable-libfdk-aac  \
            --enable-libfreetype \
            --enable-libmp3lame  \
            --enable-libopus     \
            --enable-libtheora   \
            --enable-libvorbis   \
            --enable-libvpx      \
            --enable-libx264     \
            --enable-libx265     \
            --enable-openssl     \
            --docdir=/usr/share/doc/ffmpeg-4.4.3
RUN make
RUN gcc tools/qt-faststart.c -o tools/qt-faststart

RUN make install
RUN install -v -m755    tools/qt-faststart /usr/bin
RUN install -v -m755 -d           /usr/share/doc/ffmpeg-5.1.1
RUN install -v -m644    doc/*.txt /usr/share/doc/ffmpeg-5.1.1

# Install Pangolin
WORKDIR "/home"
RUN git clone --recursive https://github.com/stevenlovegrove/Pangolin.git
WORKDIR "/home/Pangolin"
RUN sed -i '/FF_API_XVMC/,+3d' components/pango_video/include/pangolin/video/drivers/ffmpeg_common.h
RUN apt-get install -y libjpeg-dev libpng-dev libavcodec-dev libavutil-dev libavformat-dev libswscale-dev libavdevice-dev
RUN cmake -B build
RUN cmake --build build

# Install OpenCV
WORKDIR "/home"
RUN apt-get install -y python3-pip libgtk2.0-dev pkg-config
RUN pip3 install numpy
RUN wget -O opencv.zip https://github.com/opencv/opencv/archive/4.6.0.zip
RUN unzip opencv.zip 
RUN mkdir -p opencv-build
WORKDIR "/home/opencv-build"
RUN cmake -DBUILD_TESTS=OFF -DBUILD_PERF_TESTS=OFF -DBUILD_EXAMPLES=OFF  ../opencv-4.6.0
RUN cmake --build .
RUN make install

# Install G2O
WORKDIR "/home"
RUN apt-get install -y libeigen3-dev libsuitesparse-dev qtdeclarative5-dev qt5-qmake
RUN git clone https://github.com/RainerKuemmerle/g2o
WORKDIR "/home/g2o"
RUN mkdir -p build
WORKDIR "/home/g2o/build"
RUN cmake ../
RUN make
RUN make install

# Install Sophus
WORKDIR "/home"
RUN git clone https://github.com/strasdat/Sophus
WORKDIR "/home/Sophus"
RUN mkdir -p build
WORKDIR "/home/Sophus/build"
RUN cmake ../
RUN make
RUN make install

RUN export LD_LIBRARY_PATH=/lib:/usr/lib:/usr/local/lib

# Install MOV-SLAM
WORKDIR "/home"
RUN apt-get remove -y libavcodec-dev libavutil-dev
RUN apt-get autoremove -y
RUN mkdir -p /home/MOV_SLAM
COPY . /home/MOV_SLAM/
WORKDIR "/home/MOV_SLAM"
RUN ./build.sh

WORKDIR "/home"
