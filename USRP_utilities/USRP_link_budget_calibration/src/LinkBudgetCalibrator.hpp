#ifndef LINKBUDGETCALIBRATOR
#define LINKBUDGETCALIBRATOR

    //c standard library
    #include <iostream>
    #include <cstdlib>
    #include <string>
    #include <complex>
    #include <csignal>
    #include <thread>

    //JSON class
    #include <nlohmann/json.hpp>

    //source libraries
    #include "JSONHandler.hpp"
    #include "USRPHandler.hpp"
    #include "BufferHandler.hpp"
    #include "EnergyDetector.hpp"
    #include "ToneGenerator.hpp"

    using json = nlohmann::json;
    using USRPHandler_namespace::USRPHandler;
    using Buffers::Buffer_2D;
    using Buffers::Buffer_1D;
    using EnergyDetector_namespace::EnergyDetector;
    using ToneGenerator_namespace::ToneGenerator;

    namespace LinkBudgetCalibrator_namespace{

        template<typename data_type>
        class LinkBudgetCalibrator{
            private:

                //configuration variables
                json config;

                //USRP handler
                USRPHandler<data_type> usrp_handler;

                //Energy Detector
                EnergyDetector<data_type> energy_detector;

                //Tone Generator
                ToneGenerator<data_type> tone_generator;

                //debug and enable statuses
                bool debug;
                bool tone_transmitter_enabled;
                bool energy_detector_enabled;

                //path to save debug information
                std::string save_file_path;

            public:

                /**
                 * @brief Construct a new Link Budget Calibrator object- unitialized configuration
                 * 
                 */
                LinkBudgetCalibrator(){}
                

                /**
                 * @brief Construct a new Link Budget Calibrator object- Initialized
                 * 
                 * @param config_data a json object containing configuration information
                 * @param run (optional) on true, automatically runs the link budget calibration
                 */
                LinkBudgetCalibrator(
                    json config_data,
                    bool run = false)
                {
                    init(config_data,run);
                    return;
                }

                /**
                 * @brief Initialize the link budget calibrator object
                 * 
                 * @param config_data - json object containing configuration information
                 * @param run - on True, immediately runs the link budget calibration
                 */
                void init(
                    json & config_data,
                    bool run = false)
                {
                    config = config_data;

                    if (check_config())
                    {
                        usrp_handler.init(config);

                        init_debug();
                        init_energy_detector();
                        init_tone_transmitter();

                        if(run){
                            run_calibration();
                        }
                    } else{
                        std::cerr << "LinkBudgetCalibrator::init: check configuration failed" << std::endl;
                    }
                    return;
                }
            
            private: //functions to support initialization are private
                
                /**
                 * @brief Check the configuration status to make sure that the json file has the required information
                 * 
                 * @return true - configuration is good
                 * @return false 
                 */
                bool check_config(){
                    bool config_good = true;

                    //check debug status
                    if(config["LinkBudgetCalibratorSettings"]["debug"].is_null()){
                        std::cerr << "LinkBudgetCalibrator::check_config: debug not specified" <<std::endl;
                        config_good = false;
                    }

                    //save file path for saving debug information
                    if(config["LinkBudgetCalibratorSettings"]["save_file_path"].is_null()){
                        std::cerr << "LinkBudgetCalibrator::check_config: save_file_path not specified" <<std::endl;
                        config_good = false;
                    }

                    //save file path for saving debug information
                    if(config["EnergyDetectorSettings"]["enabled"].is_null()){
                        std::cerr << "LinkBudgetCalibrator::check_config: Energy Detector enable not specified" <<std::endl;
                        config_good = false;
                    }

                    //check enable status of tone transmitter
                    if(config["ToneTransmitterSettings"]["enabled"].is_null()){
                        std::cerr << "LinkBudgetCalibrator::check_config: Tone Transmitter enable status not specified" <<std::endl;
                        config_good = false;
                    }

                    return config_good;
                }

                /**
                 * @brief Initialize the debug status
                 * 
                 */
                void init_debug(){

                    //set debug status
                    debug = config["LinkBudgetCalibratorSettings"]["debug"].get<bool>();

                    //set the save file path
                    save_file_path = config["LinkBudgetCalibratorSettings"]["save_file_path"].get<std::string>();

                    return;
                }

                void init_energy_detector(){
                    energy_detector_enabled = config["EnergyDetectorSettings"]["enabled"].get<bool>();
                    if(energy_detector_enabled){
                        energy_detector.init(config);
                    }
                }
                
                /**
                 * @brief Initialize the tone transmitter
                 * 
                 */
                void init_tone_transmitter(){

                    //set enable status
                    tone_transmitter_enabled = config["ToneTransmitterSettings"]["enabled"].get<bool>();

                    //initialize the tone generator
                    //tone_generator.init(config);

                    return;
                }

            public: //function to run calibration is public

                void run_calibration(){
                    measure_signal_power();
                }

            private:

                /**
                 * @brief Measure the signal power using the energy detector
                 * 
                 */
                void measure_signal_power(){

                    usrp_handler.rx_stream_to_buffer( & energy_detector.noise_power_measureent_signal);

                    energy_detector.compute_relative_noise_power();

                    if (debug)
                    {
                        std::cout << "SensingSubsystem::measure_relative_noise_power: measuring relative noise power" << std::endl;
                        std::cout << "relative noise power: " << energy_detector.relative_noise_power << "dB" << std::endl;


                        //save the signal to a file so that its available for further processing
                        std::string path = save_file_path + "energy_detector_received_signal.bin";
                        energy_detector.noise_power_measureent_signal.set_write_file(path,true);
                        energy_detector.noise_power_measureent_signal.save_to_file();
                    }
                    
                }

        };

    }


#endif