#ifndef TONEGENERATOR
#define TONEGENERATOR

    //C standard libraries
    #include <iostream>
    #include <cstdlib>
    #include <string>
    #include <complex>
    #include <vector>
    #include <csignal>

    //uhd specific libraries
    #include <uhd/exception.hpp>
    #include <uhd/types/tune_request.hpp>
    #include <uhd/usrp/multi_usrp.hpp>
    #include <uhd/utils/safe_main.hpp>
    #include <uhd/utils/thread.hpp>
    #include <boost/format.hpp>
    #include <boost/lexical_cast.hpp>
    #include <boost/program_options.hpp>

    #define _USE_MATH_DEFINES
    #include <cmath>

    //include the JSON handling capability
    #include <nlohmann/json.hpp>
    #include "BufferHandler.hpp"

    using json = nlohmann::json;
    using namespace Buffers;

    namespace ToneGenerator_namespace{
        template<typename data_type>
        class ToneGenerator{
            
            private:
                //configuration information
                json config;

                //USRP parameters
                double stream_start_time;
                data_type sampling_frequency_Hz;
                size_t samples_per_buffer;

                //parameters for the tone buffer
                size_t num_samples_tone_signal;
                size_t num_rows_tone_signal;
                data_type tone_baseband_frequency_Hz;
                bool load_signal_from_file;
                std::string tone_signal_folder_path;
                data_type tone_period_ms;

                //parameters for constructing tones and frames
                double frame_periodicity_ms;
                size_t num_frames;
                

            public:
                Buffers::Buffer_2D<std::complex<data_type>> tone_buffer;
                std::vector<uhd::time_spec_t> frame_start_times;

                
                ToneGenerator(){}

                ToneGenerator(json & json_config){
                    // initialize the tone generator
                    init(json_config);
                }

                void init(json & json_config){
                    
                    //save the config file
                    config = json_config;

                    if (check_config())
                    {
                        initialize_USRP_params();
                    }
                    

                }
            
            private: //functions to support initialization
                
                /**
                 * @brief Check the json config file to make sure that all necessary parameters are included
                 * 
                 * @return true - JSON is all good and has required elements
                 * @return false - JSON is missing certain fields
                 * 
                 */
                bool check_config(){
                    bool config_good = true;
                    
                    //stream start times
                    if(config["USRPSettings"]["Multi-USRP"]["stream_start_time"].is_null()){
                        std::cerr << "EnergyDetector::check_config: no stream_start_time in JSON" <<std::endl;
                        config_good = false;
                    }
                    
                    //check sampling rate
                    if(config["USRPSettings"]["Multi-USRP"]["sampling_rate"].is_null()){
                        std::cerr << "EnergyDetector::check_config: no sampling_rate in JSON" <<std::endl;
                        config_good = false;
                    }

                    //check for samples per buffer
                    if(config["USRPSettings"]["RX"]["spb"].is_null()){
                        std::cerr << "EnergyDetector::check_config: spb for Rx not specified" <<std::endl;
                        config_good = false;
                    }
                    
                    //check for tone frequency
                    if(config["ToneTransmitterSettings"]["tone_baseband_frequency_Hz"].is_null()){
                        std::cerr << "EnergyDetector::check_config: tone_baseband_frequency_Hz not specified" <<std::endl;
                        config_good = false;
                    }

                    //check for load_signal_from_file
                    if(config["ToneTransmitterSettings"]["load_signal_from_file"].is_null()){
                        std::cerr << "EnergyDetector::check_config: load_signal_from_file not specified" <<std::endl;
                        config_good = false;
                    }

                    //check for tone_signal_folder_path
                    if(config["ToneTransmitterSettings"]["tone_signal_folder_path"].is_null()){
                        std::cerr << "EnergyDetector::check_config: tone_signal_folder_path not specified" <<std::endl;
                        config_good = false;
                    }

                    //check for tone_period_ms
                    if(config["ToneTransmitterSettings"]["tone_period_ms"].is_null()){
                        std::cerr << "EnergyDetector::check_config: tone_period_ms not specified" <<std::endl;
                        config_good = false;
                    }

                    //check for frame_periodicity_ms
                    if(config["ToneTransmitterSettings"]["frame_periodicity_ms"].is_null()){
                        std::cerr << "EnergyDetector::check_config: frame_periodicity_ms not specified" <<std::endl;
                        config_good = false;
                    }

                    //check for num_frames
                    if(config["ToneTransmitterSettings"]["num_frames"].is_null()){
                        std::cerr << "EnergyDetector::check_config: num_frames not specified" <<std::endl;
                        config_good = false;
                    }

                    return config_good;
                }


            void initialize_USRP_params(){

                //stream start time
                stream_start_time = config["USRPSettings"]["Multi-USRP"]["stream_start_time"].get<double>();
                
                //sampling frequency
                sampling_frequency_Hz = config["USRPSettings"]["Multi-USRP"]["sampling_rate"].get<data_type>();
                
                //samples per buffer
                samples_per_buffer = config["USRPSettings"]["RX"]["spb"];

                return;
            }


            void initialize_tone_signal(){

                //initialize the tone buffer
                initialize_tone_buffer();
                
                //determine if generating a tone or loading one from a file
                load_signal_from_file = config["ToneTransmitterSettings"]["load_signal_from_file"].get<bool>();

                if (load_signal_from_file)
                {
                    load_tone_from_file();
                }
                else{
                    compute_tone_signal();
                }

                //compute the frame start times
                initialize_frame_start_times();

                return;
            }



            void initialize_tone_buffer(){
                //determine the number of rows and samples in the tone signal
                data_type row_period_s = static_cast<data_type>(samples_per_buffer)/sampling_frequency_Hz;
                tone_period_ms = config["ToneTransmitterSettings"]["tone_period_ms"].get<data_type>();
                num_rows_tone_signal = 
                    static_cast<size_t>(std::ceil((tone_period_ms * 1e-3)/row_period_s));

                //determine the number of samples for the whole tone
                num_samples_tone_signal = num_rows_tone_signal * samples_per_buffer;

                //initialize the tone buffer
                tone_buffer = Buffers::Buffer_2D<std::complex<data_type>>(num_rows_tone_signal,samples_per_buffer);
            }



            void load_tone_from_file(){
                
                //get the path where the file is located
                tone_signal_folder_path = config["ToneTransmitterSettings"]["tone_signal_folder_path"].get<std::string>();

                //set the read file on the buffer
                tone_buffer.set_read_file(tone_signal_folder_path,true);

                //save the data to a vector
                std::vector<data_type> data = tone_buffer.load_data_from_read_file();

                //import the data into the buffer
                tone_buffer.load_data_into_buffer(data,true);
            }



            void compute_tone_signal(){
                tone_baseband_frequency_Hz = config["ToneTransmitterSettings"]["tone_baseband_frequency_Hz"].get<data_type>();

                //initialize the times at which to compute the tone at
                std::vector<data_type> t_sec(num_samples_tone_signal);
                for (size_t i = 0; i < num_samples_tone_signal; i++)
                {
                    t_sec[i] = static_cast<data_type>(i) / sampling_frequency_Hz;
                }

                std::vector<std::complex<data_type>> tone_signal(num_samples_tone_signal);
                for (size_t i = 0; i < num_samples_tone_signal; i++)
                {
                    tone_signal[i] = 
                        std::exp(
                            std::complex<data_type>(0,
                                2.0 * M_PI * tone_baseband_frequency_Hz * t_sec[i])
                        );
                }
                
                //save the generated tone to the buffer
                tone_buffer.load_data_into_buffer(tone_signal,true);
            }



            void initialize_frame_start_times(){
                
                //get the number of frames
                num_frames = config["ToneTransmitterSettings"]["num_frames"].get<size_t>();

                //set the frame periodicity
                frame_periodicity_ms = config["ToneTransmitterSettings"]["frame_periodicity_ms"].get<double>();

                //initialize the frame start times vector
                frame_start_times = std::vector<uhd::time_spec_t>(num_frames);

                for (size_t i = 0; i < num_frames; i++)
                    {
                        frame_start_times[i] = uhd::time_spec_t(stream_start_time + 
                                        (frame_periodicity_ms * 1e-3 * static_cast<double>(i)));
                    }
                return;
            }

        };
    }

#endif