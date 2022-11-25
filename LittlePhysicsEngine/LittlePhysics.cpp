#include <iostream>
#include "LittlePhysics.h"

namespace LP {

	void __declspec(dllexport) Print()
	{
		std::cout << "Hello Demo" << std::endl;
	}

}

