output = np.zeros((h,w), dtype="uint8")
ONFIG_PATH=/usr/local/lib/pkgconfig:${PKG_CONFIG_PATH}
	#export PKG_CONFIG_PATH

all:
	        g++ -o contourHand contourHand.cpp  `pkg-config --cflags --libs opencv`

#clean:		rm contourHand
