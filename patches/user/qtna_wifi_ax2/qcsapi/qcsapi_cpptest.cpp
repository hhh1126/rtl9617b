#include <iostream>
extern "C" {
#include "qcsapi.h"
}

int main(int argc, char **argv)
{
	unsigned int status;

	std::cout << qcsapi_get_system_status(&status) << std::endl;
	std::cout << "done" << std::endl;

	return 0;
}
