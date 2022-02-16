#include "common/trains_config.h"
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <vector>

bool solver_annealing(const TrainsConfig&);
int write_test_graph();
int write_dijkstra();
int test_custom_set();

int main(int argc, const char** argv)
{
    fprintf(stderr, "Build: %s %s %s\n", BUILD_TYPE, __DATE__, __TIME__);
    if(argc != 3)
    {
        fprintf(stderr, "\nSyntax: ./trains_solver <ALGO#> <INPUT_FILE>\n");
        return EXIT_FAILURE;
    }
    try
    {
        const auto algo_num = std::atoi(argv[1]);
        const std::string fpath = argv[2];
        TrainsConfig cfg(fpath);
        bool result = false;
        switch(algo_num)
        {
            case 0: result = test_custom_set(); break;
            case 2: result = solver_annealing(cfg); break;
            case 98: result = write_test_graph(); break;
            case 99: result = write_dijkstra(); break;
            default: {
                fprintf(stderr, "Unknown algo: %s\n", argv[1]);
            } break;
        }
        if(result)
        {
            fprintf(stderr, "Success.\n");
            return EXIT_SUCCESS;
        }
        else
        {
            fprintf(stderr, "Failed.\n");
            return EXIT_FAILURE;
        }
    }
    catch(const std::exception& ex)
    {
        fprintf(stderr, "Exception: %s\n", ex.what());
        return EXIT_FAILURE;
    }
}
