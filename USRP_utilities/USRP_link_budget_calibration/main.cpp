//C standard libraries
#include <iostream>
#include <cstdlib>
#include <string>
#include <chrono>
#include <complex>
#include <csignal>
#include <thread>

//uhd specific libraries
#include <uhd/exception.hpp>
#include <uhd/types/tune_request.hpp>
#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/utils/safe_main.hpp>
#include <uhd/utils/thread.hpp>
#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/program_options.hpp>

//JSON class
#include <nlohmann/json.hpp>

//source libraries
#include "src/JSONHandler.hpp"
#include "src/USRPHandler.hpp"
#include "src/BufferHandler.hpp"
#include "src/LinkBudgetCalibrator.hpp"

//set namespaces
using json = nlohmann::json;
using USRPHandler_namespace::USRPHandler;
using Buffers::Buffer;
using Buffers::Buffer_1D;
using LinkBudgetCalibrator_namespace::LinkBudgetCalibrator;


int UHD_SAFE_MAIN(int argc, char* argv[]) {

    //configuration
    std::string config_file = "/home/david/Documents/BlackBoxRadarAttacks/USRP_utilities/USRP_link_budget_calibration/config.json";


    //read the config file
    std::cout << "\nMAIN: Parsing JSON\n";
    json config = JSONHandler::parse_JSON(config_file,false);

    //check to make sure that the config has a valid format:
    if(config["USRPSettings"]["Multi-USRP"]["type"].is_null() ||
        config["USRPSettings"]["Multi-USRP"]["cpufmt"].is_null()){
            std::cerr << "MAIN: type or cpu format is not specified in config file" <<std::endl;
            return EXIT_FAILURE;
    }

    //determine that there is a valid cpu format and type
    
    std::string type = config["USRPSettings"]["Multi-USRP"]["type"].get<std::string>();
    std::string cpufmt = config["USRPSettings"]["Multi-USRP"]["cpufmt"].get<std::string>();

    
    if (type == "double" && cpufmt == "fc64"){
        LinkBudgetCalibrator<double> link_budget_calibrator(config,true);
    }
    else if (type == "float" && cpufmt == "fc32")
    {
        LinkBudgetCalibrator<float> link_budget_calibrator(config,true);
    }
    else{
        std::cerr << "MAIN: type and cpufmt don't match valid combination (must use float or double)" << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
