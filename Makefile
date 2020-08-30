all: release

CXXFLAGS_RELEASE:=	-O2 \
					-Wall \
					-Werror
CXXFLAGS_DEBUG:=	-O0 \
					-Wall \
					-ggdb3 \
					-DUNO_TRANSPORT_DEBUG

release:
	$(CXX) UnoTransport.cpp server.cpp -o server $(CXXFLAGS_RELEASE)
	$(CXX) UnoTransport.cpp server.cpp -o server $(CXXFLAGS_RELEASE)

debug:
	$(CXX) UnoTransport.cpp server.cpp -o server $(CXXFLAGS_DEBUG)
	$(CXX) UnoTransport.cpp server.cpp -o server $(CXXFLAGS_DEBUG)

clean:
	@rm -rf server
	@rm -rf client