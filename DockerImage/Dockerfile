FROM ubuntu:24.04

 # Install necessary dependencies
RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y \
    build-essential \
    cmake \
    pkg-config \
    libbz2-dev \
    libstxxl-dev \
    libstxxl1v5 \
    libxml2-dev \
    libzip-dev \
    libzstd-dev \
    libssl-dev \
    libboost-all-dev \
    lua5.2 \
    liblua5.2-dev \
    libtbb-dev \
    libmongoc-dev \
    git \
    g++-14 \
    libomp-dev \
    libpthread-stubs0-dev \
    curl \
    && rm -rf /var/lib/apt/lists/*

# Set up alternatives to use the new compiler version
RUN update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-14 100 \
    && update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-14 100

# Set the default compiler alternatives
RUN update-alternatives --set gcc /usr/bin/gcc-14 \
    && update-alternatives --set g++ /usr/bin/g++-14

# Build and install jsoncpp && mongocxx from source
WORKDIR /tmp

RUN git clone --branch 1.9.5 --depth 1 https://github.com/open-source-parsers/jsoncpp.git && \
    mkdir -p jsoncpp/build && cd jsoncpp/build && \
    cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_STATIC_LIBS=ON -DBUILD_SHARED_LIBS=ON -DARCHIVE_INSTALL_DIR=. -G "Unix Makefiles" .. && \
    make -j8 && make install && \
    rm -rf /tmp/jsoncpp

WORKDIR /app

WORKDIR /app/osrm-backend/
# Clone OSRM repository
RUN git clone https://github.com/Project-OSRM/osrm-backend.git . && \
    git checkout v6.0.0

WORKDIR /app/osrm-backend/build

# Build and install OSRM
RUN cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS="-w" ..
RUN cmake --build . --parallel 1
RUN cmake --build . --target install

WORKDIR /app
# Copy your application source code

ADD cmake ./cmake/
ADD CMakeLists.txt ./
ADD src ./src/
ADD include ./include/
ADD OSM ./OSM/

# If the PBF is present, extract and contract it to produce the .osrm files.
# We use the car profile from the cloned osrm-backend sources.
RUN if [ -f /app/OSM/region.osm.pbf ]; then \
            echo "Found region.pbf, running osrm-extract and osrm-contract..." && \
            osrm-extract -p /app/osrm-backend/profiles/car.lua /app/OSM/region.osm.pbf && \
            osrm-contract /app/OSM/region.osrm ; \
        else \
            echo "No /app/OSM/region.osm.pbf found, skipping extract/contract step." ; \
        fi
ADD results ./results/

WORKDIR /app/build

# Build your application
RUN cmake -DCMAKE_BUILD_TYPE=Release ..
RUN cmake --build . --parallel 8


# Set the entry point to execute the application with arguments
ENTRYPOINT ["/app/build/osrm", \
            "--osrm-path", "/app/OSM/region.osrm", \
            "--coordinates-path", "/app/results/coordinates.txt"]
