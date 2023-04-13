//C standard libraries
#include <iostream>
#include <cstdlib>
#include <string>
#include <chrono>
#include <complex>
#include <csignal>
#include <thread>

//src libraries
#include "src/BufferHandler.hpp"
#include "src/CrossCorr.hpp"


//set namespaces
using Buffers::Buffer_1D;
using CrossCorr_namespace::CrossCorr;

int main(int, char**) {
    
    //declare the relevant file paths
    std::string estimated_chirp_path("/home/david/Documents/BlackBoxRadarAttacks/CPP Development_Archive/cross_corr/estimated_chirp.bin");
    std::string computed_chirp_path("/home/david/Documents/BlackBoxRadarAttacks/CPP Development_Archive/cross_corr/computed_chirp.bin");

    //create the buffers
    Buffer_1D<std::complex<float>> estimated_chirp;
    Buffer_1D<std::complex<float>> computed_chirp;

    //set the read file paths
    estimated_chirp.set_read_file(estimated_chirp_path,true);
    computed_chirp.set_read_file(computed_chirp_path,true);

    //read the saved data
    estimated_chirp.import_from_file();
    computed_chirp.import_from_file();

    //print a preview of the estimated chirp to confirm import was successful
    estimated_chirp.print_preview();

    //TODO: Perform Cross Correlation
    size_t max_lag = 400;
    CrossCorr<float> cross_corr(max_lag,true);
    cross_corr.compute(estimated_chirp.buffer,computed_chirp.buffer);

    //get the delay in samples and the delay in us
    float sample_rate_MSps = 29.536;
    int delay_in_samples = cross_corr.delay_samples;
    float delay_in_us = cross_corr.compute_delay_us(sample_rate_MSps);

    std::cout << "Delay in samples: " << delay_in_samples << std::endl;
    std::cout << "Delay in us: " << delay_in_us << std::endl;

    //save computed cross correlation to a file
    std::string result_path("/home/david/Documents/BlackBoxRadarAttacks/CPP Development_Archive/cross_corr/result.bin");
    Buffer_1D<std::complex<float>> result = cross_corr.result;
    result.set_write_file(result_path);
    result.save_to_file();
}
