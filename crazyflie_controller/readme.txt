controller.cpp has two modes:
	1) goal follower
	2) goal follower + error control 

	-> activate 2) through CMakeLists.txt by adding compiler definition ERROR_INPUT

VICON_OFF is the compiler definition that removes the dependency on position information of Crazyflies
	-> consider adding this definition if testing without actual flight
