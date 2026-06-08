CXX = g++
CXXFLAGS = -std=c++17 -I /usr/include/eigen3 -I . -DSPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_TRACE
LIBS = -lfmt -lspdlog

SRCS = control.cpp pose.cpp world.cpp utils.cpp test.cpp robot.cpp landmark.cpp point2d.cpp
OBJS = $(SRCS:.cpp=.o)

test: 	$(OBJS)
	$(CXX) $(CXXFLAGS) $(OBJS) -o test $(LIBS)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm -f $(OBJS) test
