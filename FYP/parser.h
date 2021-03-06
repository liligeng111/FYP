#include <iostream>
#include <fstream>
#include <sstream>

#include "object.h"
#include "light.h"
#include "scene.h"
using namespace std;

#ifdef _WIN32 || _WIN64
    #define SLASH '/'
#else
    #define SLASH '/'
#endif

namespace Parser {
    Scene* parseScene(std::string filename);
    
	/* parse obj file for object geometry */
	Scene* parseObjFile(std::string filename);

	/* parse mtl file for material */
	void parseMtlFile(std::string filename, Scene* scene);
    
    // std::vector<std::string> split(std::string str, std::string del);
};

