#include "common/trains_config.h"
#include <stdlib.h>

int main(int argc, const char* argv[])
{
    if(argc != 2)
    {
        return EXIT_FAILURE;
    }
    std::string problem_path = argv[1];
    TrainsConfig cfg(problem_path);
    cfg.save_graphviz_dot_file(problem_path+".gv"); // .dot or .gv preferably
    return EXIT_SUCCESS;
}
