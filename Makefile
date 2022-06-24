graphlib:
	@echo "Installing up graphlib..."
	apt-get install -y libsdl-image1.2 libsdl-image1.2-dev guile-2.0 \
        guile-2.0-dev libsdl1.2debian libart-2.0-dev libaudiofile-dev \
        libesd0-dev libdirectfb-dev libdirectfb-extra libfreetype6-dev \
        libxext-dev x11proto-xext-dev libfreetype6 libaa1 libaa1-dev \
        libslang2-dev libasound2 libasound2-dev

	wget $(LIBGRAPH_URL)
	tar -xf $(LIBGRAPH_DIRECTORY).tar.gz
	apt install -y guile-2.0-dev  # In case you didn't install it earlier
	cd $(LIBGRAPH_DIRECTORY); CPPFLAGS="$CPPFLAGS $(pkg-config --cflags-only-I guile-2.0)" \
    	CFLAGS="$CFLAGS $(pkg-config --cflags-only-other guile-2.0)" \
      	LDFLAGS="$LDFLAGS $(pkg-config --libs guile-2.0)"
	cd $(LIBGRAPH_DIRECTORY); ./configure --disable-guile; make; make install; cp /usr/local/lib/libgraph.* /usr/lib

	rm -rf $(LIBGRAPH_DIRECTORY).tar.gz
	rm -rf $(LIBGRAPH_DIRECTORY)

	@echo "graphlib has been installed."

dependencies:
	@echo "Installing  dependencies..."
	make graphlib
	@echo "dependencies have been installed."


build:
	g++ main.cpp -o $(BINARY_NAME) -lgraph

clean:
	@echo "Cleaning..."
	-rm -f $(BINARY_NAME)
	-rm -f $(BINARY_UNIX)
	@echo "Cleaning done."

docker-build:
	docker build -t $(DOCKER_IMAGE_NAME):latest .

build-by-docker:
	docker run --rm --name builder -it --user $$(id -u):$$(id -g) $(DOCKER_VOLUMES) $(DOCKER_IMAGE_NAME) build

run:
	./$(BINARY_NAME)


BUILD_DIRECTORY=build
BUILD_PATH=TSP/
PROJECT_PATH=TSP/
LIBGRAPH_URL=http://download.savannah.gnu.org/releases/libgraph/libgraph-1.0.2.tar.gz
LIBGRAPH_DIRECTORY=libgraph-1.0.2
BUILD_PROTO_DIRECTORY=../
BINARY_NAME=TSP
BINARY_UNIX=$(BINARY_NAME)_unix
DOCKER_VOLUMES=-v $$PWD:/$(PROJECT_PATH)
DOCKER_IMAGE_NAME=ap:8000/tsp
