#ifndef ATTACKINGSUBSYSTEM
#define ATTACKINGSUBSYSTEM

    //c standard library
    #include <iostream>
    #include <cstdlib>
    #include <string>
    #include <complex>
    #include <csignal>
    #include <mutex>
    #include <thread>

    //to track processing times
    #include <chrono>

    //JSON class
    #include <nlohmann/json.hpp>

    //source libraries
    #include "../JSONHandler.hpp"
    #include "../USRPHandler.hpp"
    #include "../BufferHandler.hpp"

    // add in namespaces as needed
    using json = nlohmann::json;
    using USRPHandler_namespace::USRPHandler;
    using Buffers::Buffer_2D;
    using Buffers::Buffer_1D;
    using Buffers::FMCW_Buffer;

    //Variable for tracking process times
    using namespace std::chrono;

    namespace AttackingSubsystem_namespace{

        template<typename data_type>
        class AttackingSubsystem {
            public:
                //enable flag
                bool enabled;

                //attack status
                bool attacking; //true when the system is attacking

                //attack complete (for sensing system to tell attacking subsystem that the attack is complete)
                bool sensing_complete;
            private:
                //pointer to usrp device
                USRPHandler<data_type> * attacker_usrp_handler;

                //configuration
                json config;

                //timing arguments
                double attack_start_time_ms;
                double stream_start_offset_us;
                std::vector<uhd::time_spec_t> frame_start_times;

                //for tracking how many frames have been loaded and the current attacking frame
                size_t current_attack_frame; //for tracking the current attack frame
                size_t next_frame_to_load; //for tracking the next frame to load
                bool new_frame_start_time_available;
                
                //FMCW arguments
                size_t num_attack_frames;

                //mutexes to ensure thread safety
                std::mutex frame_start_times_mutex;
                std::mutex sensing_complete_mutex;
                std::mutex victim_parameters_mutex;

            public:
                size_t attack_start_frame;
            private:

                //RF parameters for the attackers and support parameters
                const double c_m_s = 2.99792458e8;
                double lambda_m; 
                double FMCW_sampling_rate_Hz;
                double FMCW_sampling_period_s;
                
                //estimated victim parameters used to compute victim waveform
                double estimated_chirp_cycle_time_us;
                double estimated_frequency_slope_MHz_us;
                double estimated_frame_periodicity_ms;
                size_t chirps_per_frame;

                //calculated victim parameters based on estimated parameters
                double sweep_time_us;
                double idle_time_us;
                size_t num_samples_per_chirp;
                size_t num_samples_sweep_time;
                size_t num_samples_idle_time;

                //variable to hold times at which to compute chirp signal
                std::vector<double> t_sec;
            
            public: //make victim waveform public so that it can be accessed by sensing subsystem
                //variable to store the estimated victim waveform - used by the sensing subsystem
                Buffer_1D<std::complex<data_type>> victim_waveform;
                bool victim_waveform_loaded;

            private: //make variables relevant to the attack private
                //status variable for when victim parameters have been estimated
                bool victim_parameters_loaded;
                
                //variables to store the current victim position 
                //(variables are scalar quantities containing the relative position and velocity)
                double current_victim_pos_m;
                double current_victim_vel_m_s;

                //variables to control false negative attacks
                //TODO: add functionality to support these variables
                double current_FN_spoofing_pos_m;
                double current_FN_spoofing_vel_m_s;
                bool FN_spoof_enable;
                bool sim_vel_attack_enable;
                bool sim_slope_attack_enable;


                //variables to store false positive spoofing settings
                std::vector<double> current_FP_spoofing_positions_m;
                std::vector<double> current_FP_spoofing_velocities_m_s;
                bool FP_spoof_enable;

                //variables to track the number of attack signals being generated
                size_t num_spoofing_signals;

                //attack signal buffer
                double samples_per_buffer;
                FMCW_Buffer<data_type> USRP_attack_signal_buffer;
                Buffer_2D<std::complex<data_type>> attack_chirps_buffer;

                //debug variable
                bool debug;

            public:
                
                /**
                 * @brief Construct a new Attacking Subsystem object - DEFAULT CONSTRUCTOR, DOES NOT INITIALIZE AttackingSubsystem
                 * 
                 */
                AttackingSubsystem(){}

                /**
                 * @brief Construct a new Attacking Subsystem object
                 * 
                 * @param config_data JSON configuration object for the attacker
                 * @param usrp_handler pointer to a USRP handler for the attacking subsystem to use
                 */
                AttackingSubsystem(json config_data, USRPHandler<data_type> * usrp_handler){
                    // initialize the Attacking Subsystem
                    init(config_data,usrp_handler);                 
                }

                /**
                 * @brief Construct a new Attacking Subsystem object -Copy Constructor
                 * 
                 * @param rhs 
                 */
                AttackingSubsystem(const AttackingSubsystem & rhs) : enabled(rhs.enabled),
                                                                    attacking(rhs.attacking),
                                                                    sensing_complete(rhs.sensing_complete),
                                                                    attacker_usrp_handler(rhs.attacker_usrp_handler),
                                                                    config(rhs.config),
                                                                    attack_start_time_ms(rhs.attack_start_time_ms),
                                                                    stream_start_offset_us(rhs.stream_start_offset_us),
                                                                    frame_start_times(rhs.frame_start_times),
                                                                    current_attack_frame(rhs.current_attack_frame),
                                                                    next_frame_to_load(rhs.next_frame_to_load),
                                                                    new_frame_start_time_available(rhs.new_frame_start_time_available),
                                                                    num_attack_frames(rhs.num_attack_frames),
                                                                    frame_start_times_mutex(), //mutexes are not copyable
                                                                    sensing_complete_mutex(), //mutexes are not copyable
                                                                    victim_parameters_mutex(), //mutexes are not copyable
                                                                    attack_start_frame(rhs.attack_start_frame),
                                                                    c_m_s(rhs.c_m_s),
                                                                    lambda_m(rhs.lambda_m),
                                                                    FMCW_sampling_rate_Hz(rhs.FMCW_sampling_rate_Hz),
                                                                    FMCW_sampling_period_s(rhs.FMCW_sampling_period_s),
                                                                    estimated_chirp_cycle_time_us(rhs.estimated_chirp_cycle_time_us),
                                                                    estimated_frequency_slope_MHz_us(rhs.estimated_frequency_slope_MHz_us),
                                                                    estimated_frame_periodicity_ms(rhs.estimated_frame_periodicity_ms),
                                                                    chirps_per_frame(rhs.chirps_per_frame),
                                                                    sweep_time_us(rhs.sweep_time_us),
                                                                    idle_time_us(rhs.idle_time_us),
                                                                    num_samples_per_chirp(rhs.num_samples_per_chirp),
                                                                    num_samples_sweep_time(rhs.num_samples_sweep_time),
                                                                    num_samples_idle_time(rhs.num_samples_idle_time),
                                                                    t_sec(rhs.t_sec),
                                                                    victim_waveform(rhs.victim_waveform),
                                                                    victim_waveform_loaded(rhs.victim_waveform_loaded),
                                                                    victim_parameters_loaded(rhs.victim_parameters_loaded),
                                                                    current_victim_pos_m(rhs.current_victim_pos_m),
                                                                    current_victim_vel_m_s(rhs.current_victim_vel_m_s),
                                                                    current_FN_spoofing_pos_m(rhs.current_FN_spoofing_pos_m),
                                                                    current_FN_spoofing_vel_m_s(rhs.current_FN_spoofing_vel_m_s),
                                                                    FN_spoof_enable(rhs.FN_spoof_enable),
                                                                    sim_vel_attack_enable(rhs.sim_vel_attack_enable),
                                                                    sim_slope_attack_enable(rhs.sim_slope_attack_enable),
                                                                    current_FP_spoofing_positions_m(rhs.current_FP_spoofing_positions_m),
                                                                    current_FP_spoofing_velocities_m_s(rhs.current_FP_spoofing_velocities_m_s),
                                                                    FP_spoof_enable(rhs.FP_spoof_enable),
                                                                    num_spoofing_signals(rhs.num_spoofing_signals),
                                                                    samples_per_buffer(rhs.samples_per_buffer),
                                                                    USRP_attack_signal_buffer(rhs.USRP_attack_signal_buffer),
                                                                    attack_chirps_buffer(rhs.attack_chirps_buffer),
                                                                    debug(rhs.debug)
                                                                    {}


                /**
                 * @brief Assignment operator
                 * 
                 * @param rhs existing AttackingSubsystem
                 * @return AttackingSubsystem& 
                 */
                AttackingSubsystem & operator=(const AttackingSubsystem & rhs){
                    if(this != & rhs){
                        enabled = rhs.enabled;
                        attacking = rhs.attacking;
                        sensing_complete = rhs.sensing_complete;
                        attacker_usrp_handler = rhs.attacker_usrp_handler;
                        config = rhs.config;
                        attack_start_time_ms = rhs.attack_start_time_ms;
                        stream_start_offset_us = rhs.stream_start_offset_us;
                        frame_start_times = rhs.frame_start_times;
                        current_attack_frame = rhs.current_attack_frame;
                        next_frame_to_load = rhs.next_frame_to_load;
                        new_frame_start_time_available = rhs.new_frame_start_time_available;
                        num_attack_frames = rhs.num_attack_frames;
                        //both of the mutexes are not copyable
                        attack_start_frame = rhs.attack_start_frame;
                        //c_m_s is a constant and doesn't need to be set
                        lambda_m = rhs.lambda_m; 
                        FMCW_sampling_rate_Hz = rhs.FMCW_sampling_rate_Hz;
                        FMCW_sampling_period_s = rhs.FMCW_sampling_period_s;
                        estimated_chirp_cycle_time_us = rhs.estimated_chirp_cycle_time_us;
                        estimated_frequency_slope_MHz_us = rhs.estimated_frequency_slope_MHz_us;
                        estimated_frame_periodicity_ms = rhs.estimated_frame_periodicity_ms;
                        chirps_per_frame = rhs.chirps_per_frame;
                        sweep_time_us = rhs.sweep_time_us;
                        idle_time_us = rhs.idle_time_us;
                        num_samples_per_chirp = rhs.num_samples_per_chirp;
                        num_samples_sweep_time = rhs.num_samples_sweep_time;
                        num_samples_idle_time = rhs.num_samples_idle_time;
                        t_sec = rhs.t_sec;
                        victim_waveform = rhs.victim_waveform;
                        victim_waveform_loaded = rhs.victim_waveform_loaded;
                        victim_parameters_loaded = rhs.victim_parameters_loaded;
                        current_victim_pos_m = rhs.current_victim_pos_m;
                        current_victim_vel_m_s = rhs.current_victim_vel_m_s;
                        current_FN_spoofing_pos_m = rhs.current_FN_spoofing_pos_m;
                        current_FN_spoofing_vel_m_s = rhs.current_FN_spoofing_vel_m_s;
                        FN_spoof_enable = FN_spoof_enable;
                        sim_vel_attack_enable = sim_vel_attack_enable;
                        sim_slope_attack_enable = sim_slope_attack_enable;
                        current_FP_spoofing_positions_m = current_FP_spoofing_positions_m;
                        current_FP_spoofing_velocities_m_s = current_FP_spoofing_velocities_m_s;
                        FP_spoof_enable = FP_spoof_enable;
                        num_spoofing_signals = num_spoofing_signals;
                        samples_per_buffer = rhs.samples_per_buffer;
                        USRP_attack_signal_buffer = rhs.USRP_attack_signal_buffer;
                        attack_chirps_buffer = rhs.attack_chirps_buffer;
                        debug = rhs.debug;
                    }

                    return *this;
                }

                ~AttackingSubsystem(){}

                /**
                 * @brief initialize the AttackingSubsystem
                 * 
                 * @param config_data json object with configuration data
                 * @param usrp_handler pointer to a USRPHandler object
                 */
                void init(json & config_data, USRPHandler<data_type> * usrp_handler){
                    config = config_data;
                    attacker_usrp_handler = usrp_handler;
                    if (check_config())
                    {
                        //initialize debug
                        init_debug();
                        
                        //initialize key attacker parameters regardless
                        init_attack_subsystem_parameters();
                        //initialize victim parameter estimation capability regardless of whether or not attacker is enabled
                        init_estimated_parameter_values();
                        if (enabled)
                        {
                            init_frame_start_times_buffer();
                        }
                        
                        //initialize spoofing parameters
                        //TODO: replace this functionality with real time updating in the future
                        init_spoofing_parameters();
                        
                    }
                }

            private: //private helper initialization functions
                /**
                 * @brief Check the json config file to make sure all necessary parameters are included
                 * 
                 * @return true - JSON is all good and has required elements
                 * @return false - JSON is missing certain fields
                 */
                bool check_config(){
                    bool config_good = true;

                    //enable status
                    if(config["AttackSubsystemSettings"]["enabled"].is_null()){
                        std::cerr << "AttackSubsystem::check_config: no enabled in JSON" <<std::endl;
                        config_good = false;
                    }

                    //number of attack frames
                    if(config["AttackSubsystemSettings"]["num_attack_frames"].is_null()){
                        std::cerr << "AttackSubsystem::check_config: no num_attack_frames in JSON" <<std::endl;
                        config_good = false;
                    }
                    
                    //attack start frame - the frame that the attack starts on
                    if(config["AttackSubsystemSettings"]["attack_start_frame"].is_null()){
                        std::cerr << "AttackSubsystem::check_config: no attack_start_frame in JSON" <<std::endl;
                        config_good = false;
                    }
                    
                    //samples per buffer
                    if(config["USRPSettings"]["TX"]["spb"].is_null()){
                        std::cerr << "AttackSubsystem::check_config: no samples per buffer for Tx in JSON" <<std::endl;
                        config_good = false;
                    }

                    //stream start offset
                    if(config["USRPSettings"]["RX"]["offset_us"].is_null()){
                        std::cerr << "AttackSubsystem::check_config: no RX offset_us in JSON" <<std::endl;
                        config_good = false;
                    }

                    //sampling rate
                    if(config["USRPSettings"]["Multi-USRP"]["sampling_rate"].is_null()){
                        std::cerr << "AttackSubsystem::check_config: no sampling_rate in JSON" <<std::endl;
                        config_good = false;
                    }

                    //transmit frequency
                    if(config["USRPSettings"]["Multi-USRP"]["center_freq"].is_null()){
                        std::cerr << "AttackSubsystem::check_config: no center_freq in JSON" <<std::endl;
                        config_good = false;
                    }

                    //Victim relative range
                    if(config["AttackSubsystemSettings"]["current_victim_pos_m"].is_null()){
                        std::cerr << "AttackSubsystem::check_config: no current_victim_pos_m in JSON" <<std::endl;
                        config_good = false;
                    }

                    //Victim relative velocity
                    if(config["AttackSubsystemSettings"]["current_victim_vel_m_s"].is_null()){
                        std::cerr << "AttackSubsystem::check_config: no current_victim_vel_m_s in JSON" <<std::endl;
                        config_good = false;
                    }

                    //TODO: add ability to load these in from a file if possible
                    //FP spoofing enabled
                    if(config["AttackSubsystemSettings"]["FP_spoof_enable"].is_null()){
                        std::cerr << "AttackSubsystem::check_config: no FP_spoof_enable in JSON" <<std::endl;
                        config_good = false;
                    }

                    //FP spoofing distances
                    if(config["AttackSubsystemSettings"]["FP_spoof_distances_m"].is_null()){
                        std::cerr << "AttackSubsystem::check_config: no FP_spoof_distances_m in JSON" <<std::endl;
                        config_good = false;
                    }

                    //FP spoofing velocitites
                    if(config["AttackSubsystemSettings"]["FP_spoof_vels_m_s"].is_null()){
                        std::cerr << "AttackSubsystem::check_config: no FP_spoof_vels_m_s in JSON" <<std::endl;
                        config_good = false;
                    }

                    //FN spoofing enabled
                    if(config["AttackSubsystemSettings"]["FN_spoof_enable"].is_null()){
                        std::cerr << "AttackSubsystem::check_config: no FN_spoof_enable in JSON" <<std::endl;
                        config_good = false;
                    }

                    //FN spoofing distance
                    if(config["AttackSubsystemSettings"]["FN_spoof_distance_m"].is_null()){
                        std::cerr << "AttackSubsystem::check_config: no FN_spoof_distance_m in JSON" <<std::endl;
                        config_good = false;
                    }

                    //FN spoofing velocity
                    if(config["AttackSubsystemSettings"]["FN_spoof_vel_m_s"].is_null()){
                        std::cerr << "AttackSubsystem::check_config: no FN_spoof_vel_m_s in JSON" <<std::endl;
                        config_good = false;
                    }

                    //similar slope
                    if(config["AttackSubsystemSettings"]["sim_slope_attack_enable"].is_null()){
                        std::cerr << "AttackSubsystem::check_config: no sim_slope_attack_enable in JSON" <<std::endl;
                        config_good = false;
                    }

                    //similar velocity attack
                    if(config["AttackSubsystemSettings"]["sim_vel_attack_enable"].is_null()){
                        std::cerr << "AttackSubsystem::check_config: no sim_vel_attack_enable in JSON" <<std::endl;
                        config_good = false;
                    }

                    //debug status
                    if(config["AttackSubsystemSettings"]["debug"].is_null()){
                        std::cerr << "AttackSubsystem::check_config: no debug in JSON" <<std::endl;
                        config_good = false;
                    }

                    return config_good;
                }

                /**
                 * @brief Initialize attack subsystem parameters
                 * 
                 */
                void init_attack_subsystem_parameters(){
                    
                    //enabled status
                    enabled = config["AttackSubsystemSettings"]["enabled"].get<bool>();

                    //Attack configuration parameters
                    num_attack_frames = config["AttackSubsystemSettings"]["num_attack_frames"].get<size_t>();
                    attack_start_frame = config["AttackSubsystemSettings"]["attack_start_frame"].get<size_t>();

                    //USRP parameters
                    stream_start_offset_us = config["USRPSettings"]["RX"]["offset_us"].get<double>();
                    samples_per_buffer = config["USRPSettings"]["TX"]["spb"].get<double>();

                    //set attacking flags
                    attacking = false;
                    sensing_complete = false;
                }

                /**
                 * @brief Initialize the buffer that tracks the frame start times
                 * 
                 */
                void init_frame_start_times_buffer(void){
                    frame_start_times = std::vector<uhd::time_spec_t>(num_attack_frames);
                    current_attack_frame = 0;
                    next_frame_to_load = 0;
                    new_frame_start_time_available = false;
                }

                /**
                 * @brief Initializes all of the variables that store estimates of the
                 * victim's parameters and compute the estimated victim waveform
                 * 
                 */
                void init_estimated_parameter_values(void){

                    //get FMCW parameter values from JSON
                    FMCW_sampling_rate_Hz = config["USRPSettings"]["Multi-USRP"]["sampling_rate"].get<double>();
                    FMCW_sampling_period_s = 1.0 / FMCW_sampling_rate_Hz;
                    lambda_m = c_m_s / config["USRPSettings"]["Multi-USRP"]["center_freq"].get<double>();
                    
                    //init parameter estimations
                    estimated_chirp_cycle_time_us = 0;
                    estimated_frequency_slope_MHz_us = 0;

                    //init to-be calculated victim parameters
                    sweep_time_us = 0;
                    idle_time_us = 0;
                    num_samples_per_chirp = 0;
                    num_samples_sweep_time = 0;
                    num_samples_idle_time = 0;

                    //initialize t to have 100 elements, all equal to zero
                    t_sec = std::vector<double>(100,0);
                    
                    //initialize an empty 1D buffer for the victim waveform and note that it isn't loaded yet
                    victim_waveform = Buffer_1D<std::complex<data_type>>();
                    victim_waveform_loaded = false;
                    victim_parameters_loaded = false;
                }
            
                void init_spoofing_parameters(void){
                    
                    //set the victim position
                    double victim_pos_m = config["AttackSubsystemSettings"]["current_victim_pos_m"].get<double>();
                    double victim_vel_m_s =  config["AttackSubsystemSettings"]["current_victim_pos_m"].get<double>();
                    set_victim_pos_vel(victim_pos_m,victim_vel_m_s);

                    //configure FN spoofing
                    FN_spoof_enable = config["AttackSubsystemSettings"]["FN_spoof_enable"].get<bool>();
                    sim_vel_attack_enable = config["AttackSubsystemSettings"]["sim_vel_attack_enable"].get<bool>();
                    sim_slope_attack_enable = config["AttackSubsystemSettings"]["sim_slope_attack_enable"].get<bool>();
                    double FN_spoof_pos_m = config["AttackSubsystemSettings"]["FN_spoof_distance_m"].get<double>();
                    double FN_spoof_vel_m_s = config["AttackSubsystemSettings"]["FN_spoof_vel_m_s"].get<double>();
                    set_FN_spoof_pos_vel(FN_spoof_pos_m,FN_spoof_vel_m_s);

                    //configure Fp spoofing
                    FP_spoof_enable = config["AttackSubsystemSettings"]["FP_spoof_enable"].get<bool>();
                    std::vector<double> FP_spoof_pos_m = config["AttackSubsystemSettings"]["FP_spoof_distances_m"].get<std::vector<double>>();
                    std::vector<double> FP_spoof_vel_m_s = config["AttackSubsystemSettings"]["FP_spoof_vels_m_s"].get<std::vector<double>>();
                    set_FP_spoof_pos_vel(FP_spoof_pos_m,FP_spoof_vel_m_s);
                }

                void init_debug(void){
                    debug = config["AttackSubsystemSettings"]["debug"].get<bool>();
                }


            public: //the following functions are public and meant to be called by other classes
                
                void compute_calculated_values(
                    double chirp_cycle_time_us,
                    double frequency_slope_MHz_us,
                    double frame_periodicity_ms,
                    size_t num_chirps
                ){
                    
                    //lock the mutex so that the attacking thread isn't trying to read paramter estimations while updating them at the same time
                    std::unique_lock<std::mutex> victim_parameters_lock(victim_parameters_mutex, std::defer_lock);

                    //lock the mutex
                    victim_parameters_lock.lock();

                    //save estimated value
                    estimated_chirp_cycle_time_us = chirp_cycle_time_us;
                    estimated_frequency_slope_MHz_us = frequency_slope_MHz_us;
                    estimated_frame_periodicity_ms = frame_periodicity_ms;
                    chirps_per_frame = num_chirps;
                    
                    //calculated computed victim parameters
                    calculate_victim_parameters();

                    //compute values of t
                    t_sec = std::vector<double>(num_samples_per_chirp);
                    for (size_t i = 0; i < num_samples_per_chirp; i++)
                    {
                        t_sec[i] = static_cast<double>(i) * FMCW_sampling_period_s;
                    }
                    //initialize the victim waveform buffer to be the correct size
                    victim_waveform.set_buffer_size(num_samples_per_chirp);

                    //compute the victim waveform (with no time delay, phase shift, or power scaling)
                    compute_FMCW_waveform(
                        victim_waveform.buffer,
                        estimated_frequency_slope_MHz_us,
                        0,0, //phase shift
                        1.0, //power scaling
                        true); //overwrite the existing buffer

                    //set flag noting that a victim waveform has been loaded
                    victim_waveform_loaded = true;

                    victim_parameters_loaded = true;

                    //unlock the mutex - updating victim parameters now complete
                    victim_parameters_lock.unlock();
                }
                
                /**
                 * @brief Load a new frame start time into the frame_start_time_buffer
                 * 
                 * @param desired_attack_start_time_ms the desired attack start time in ms
                 */
                void load_new_frame_start_time(double desired_attack_start_time_ms){

                    std::unique_lock<std::mutex> frame_start_times_lock(frame_start_times_mutex, std::defer_lock);


                    if (next_frame_to_load < num_attack_frames)
                    {

                        //set stream start time
                        double attack_start_time_s = (desired_attack_start_time_ms - (stream_start_offset_us * 1e-3)) * 1e-3;
                        
                        //check to make sure that the attack_start_time is valid
                        double current_time = attacker_usrp_handler -> usrp -> get_time_now().get_real_secs();
                        
                        if((attack_start_time_s - current_time) <= 1e-3)
                        {
                            std::cerr << "AttackingSubsystem::load_new_frame_start_time: frame start time occurs before current USRP time" <<std::endl;
                        }
                        else
                        {
                            if(next_frame_to_load == 0)
                            {
                                attacking = true;
                            }

                            frame_start_times_lock.lock();
                            frame_start_times[next_frame_to_load] = uhd::time_spec_t(attack_start_time_s);
                            next_frame_to_load += 1;
                            new_frame_start_time_available = true;
                            frame_start_times_lock.unlock();
                        }
                    }
                }

                

                void run(){
                    //lock the mutex so that the attacking thread isn't trying to read paramter estimations while updating them at the same time
                    std::unique_lock<std::mutex> victim_parameters_lock(victim_parameters_mutex, std::defer_lock);

                    //lock the mutex
                    victim_parameters_lock.lock();
                    bool param_loaded_status = victim_parameters_loaded;
                    victim_parameters_lock.unlock();
                    
                    //wait until a victim configuration has been loaded
                    while (not get_sensing_complete())
                    {
                        if (not param_loaded_status)
                        {
                            std::this_thread::sleep_for(std::chrono::milliseconds(int64_t(1)));
                            victim_parameters_lock.lock();
                            param_loaded_status = victim_parameters_loaded;
                            victim_parameters_lock.unlock();
                        }
                        else
                        {
                            //when the parameters have been loaded, immediately compute the attack signal
                            compute_spoofing_signals();
                            USRP_attack_signal_buffer.load_chirps_into_buffer(attack_chirps_buffer.buffer);
                            break;
                        }
                    }
                    

                    while (current_attack_frame < num_attack_frames && not get_sensing_complete()) //make sure that the sensing subsystem is still active
                    {
                        std::unique_lock<std::mutex> frame_start_times_lock(frame_start_times_mutex, std::defer_lock);
                        frame_start_times_lock.lock();
                        if (new_frame_start_time_available)
                        {
                            std::vector<uhd::time_spec_t> frame_start_time(1);
                            frame_start_time[0] = frame_start_times[current_attack_frame];
                            current_attack_frame += 1;
                            if (current_attack_frame == next_frame_to_load)
                            {
                                new_frame_start_time_available = false;
                            }
                            frame_start_times_lock.unlock();

                            //transmit the attack
                            attacker_usrp_handler -> stream_frames_tx_only(frame_start_time, & USRP_attack_signal_buffer);

                            //compute the next attack signal
                            compute_spoofing_signals();

                            USRP_attack_signal_buffer.load_chirps_into_buffer(attack_chirps_buffer.buffer);
                            
                        }
                        else{
                            frame_start_times_lock.unlock();
                            //std::this_thread::sleep_for(std::chrono::milliseconds(int64_t(1)));
                        }
                    }
                    return;
                }

                /**
                 * @brief Set the sensing complete object's value
                 * 
                 * @param new_value - set to true if the sensing subsystem is no longer performing sensing
                 */
                void set_sensing_complete(bool new_value){
                    std::unique_lock<std::mutex> sensing_complete_lock(sensing_complete_mutex, std::defer_lock);
                    
                    sensing_complete_lock.lock();
                    sensing_complete = new_value;
                    sensing_complete_lock.unlock();
                }

                /**
                 * @brief Get the attack complete object's value
                 * 
                 * @return true - sensing subsystem is no longer performing sensing
                 * @return false - sensing subsystem is still sensing
                 */
                bool get_sensing_complete(){
                    std::unique_lock<std::mutex> sensing_complete_lock(sensing_complete_mutex, std::defer_lock);
                    
                    sensing_complete_lock.lock();
                    bool sensing_complete_status = sensing_complete;
                    sensing_complete_lock.unlock();

                    return sensing_complete_status;
                }

                /**
                 * @brief Resets the attacking subsystem (useful if performing multiple runs)
                 * 
                 */
                void reset(){
                    //TODO: fix this function
                    init_attack_subsystem_parameters();
                    if (enabled)
                    {
                        
                    }
                }

                /**
                 * @brief Set the current victim position and velocity
                 * 
                 * @param pos_m scalar corresponding to the relative position of the victim with regards to the attacker in meters
                 * @param vel_m_s scalar corresponding to the relative velocity of the victim with regards to the attacker in m/s
                 */
                void set_victim_pos_vel(double pos_m, double vel_m_s){
                    current_victim_pos_m = pos_m;
                    current_victim_vel_m_s = vel_m_s;
                }

                /**
                 * @brief Set the current FN spoofing position and velocity
                 * 
                 * @param pos_m a scalar corresponding to the desired position of the FN attack in meters
                 * @param vel_m_s a scalar corresponding to the desired velocity of the FN attack in meters
                 */
                void set_FN_spoof_pos_vel(double pos_m, double vel_m_s){
                    current_FN_spoofing_pos_m = pos_m;
                    current_FN_spoofing_vel_m_s = vel_m_s;
                }

                /**
                 * @brief Set the current FP spoofing positions and velocities
                 * 
                 * @param pos_m a vector containing the desired positions to launch FP spoofing attacks
                 * @param vel_m_s a vector containing the desired velocities to launch FN spoofing attacks
                 */
                void set_FP_spoof_pos_vel(std::vector<double> positions_m, std::vector<double> velocities_m_s){
                    
                    //set the FP spoofing positions/velocities
                    current_FP_spoofing_positions_m = positions_m;
                    current_FP_spoofing_velocities_m_s = velocities_m_s;
                }

            private: //the following functions are support functions intented to support the public functions above

                /**
                 * @brief Compute the the "calculated" victim parameters including
                 * num samples per chirp, sweep time, and idle time. Function also
                 * rounds timing estimates to be within an integer multiple of the 
                 * sampling period
                 * 
                 */
                void calculate_victim_parameters(){

                    //compute number of samples in a chirp
                    num_samples_per_chirp = static_cast<size_t>(
                        std::round(
                            estimated_chirp_cycle_time_us *
                            FMCW_sampling_rate_Hz * 1e-6
                        )
                    );

                    //round chirp cycle time so that it is multiple of FMCW sampling rate
                    estimated_chirp_cycle_time_us = static_cast<double>(num_samples_per_chirp) *
                        FMCW_sampling_period_s * 1e6;

                    //compute sweep time
                    sweep_time_us = (FMCW_sampling_rate_Hz * 1e-6) / estimated_frequency_slope_MHz_us;

                    //if originally computed sweep time is longer than chirp cycle time, correct it
                    //to be equal to the chirp cycle time
                    if (sweep_time_us > estimated_chirp_cycle_time_us)
                    {
                        sweep_time_us = estimated_chirp_cycle_time_us;
                    }

                    //compute the number of samples for the sweep time
                    num_samples_sweep_time = static_cast<size_t>(
                        std::floor(
                            sweep_time_us * FMCW_sampling_rate_Hz * 1e-6
                        )
                    );

                    //round the sweep time
                    sweep_time_us = static_cast<double>(num_samples_sweep_time) *
                        FMCW_sampling_period_s * 1e6;
                    
                    //compute the idle time
                    idle_time_us = estimated_chirp_cycle_time_us - sweep_time_us;
                    num_samples_idle_time = num_samples_per_chirp - num_samples_sweep_time;
                }

                /**
                 * @brief Compute an FMCW waveform for a single chirp
                 * 
                 * @param result_location pointer to a complex vector (assumed to already be initialized)
                 * where the computed waveform should be stored
                 * @param slope_MHz_us The slope of the FMCW chirp
                 * @param time_delay_s The time delay of the chirp (for spoofing at a desired range)
                 * @param phase_shift_rad Any phase shift (in radians) to apply (for spoofing velocity)
                 * @param power_scaling Power scaling for the FMCW waveform (for adjusting with respect to range)
                 * @param overwrite_result_location On True, the result location values are over-written with the newly computed FMCW chirp signals. On False, the newly computed FMCW chirp signals are added to the existing values in the result location. Defaults to True.
                 */
                void compute_FMCW_waveform(
                    std::vector<std::complex<data_type>> & result_location,
                    double slope_MHz_us,
                    double time_delay_s,
                    double phase_shift_rad,
                    data_type power_scaling,
                    bool overwrite_result_location = true){
                    
                    //variable to store the time to compute each sample at (factoring in the delay)
                    double t_actual = 0;

                    //compute the FMCW waveform for each time step in the t_sec vector
                    if(overwrite_result_location){
                        for (size_t i = 0; i < num_samples_per_chirp; i++)
                        {
                            t_actual = t_sec[i] - time_delay_s;
                            if (t_actual < 0) 
                            {
                                result_location[i] = std::complex<data_type>(0,0);
                            }
                            else if (t_actual > sweep_time_us * 1e-6)
                            {
                                result_location[i] = std::complex<data_type>(0,0);
                            }
                            else
                            {
                                result_location[i] = power_scaling * 
                                    std::exp(
                                        std::complex<data_type>(0,
                                            static_cast<data_type>(
                                                M_PI * slope_MHz_us * 1e12 *
                                                std::pow(t_actual,2) + phase_shift_rad
                                            ))
                                    );
                            }
                        }
                    } else{ //if overwrite is false, add the computed signal to the existing signal in the buffer
                        for (size_t i = 0; i < num_samples_per_chirp; i++)
                        {
                            t_actual = t_sec[i] - time_delay_s;
                            if (t_actual < 0) 
                            {
                                result_location[i] += std::complex<data_type>(0,0);
                            }
                            else if (t_actual > sweep_time_us * 1e-6)
                            {
                                result_location[i] += std::complex<data_type>(0,0);
                            }
                            else
                            {
                                result_location[i] += power_scaling * 
                                    std::exp(
                                        std::complex<data_type>(0,
                                            static_cast<data_type>(
                                                M_PI * slope_MHz_us * 1e12 *
                                                std::pow(t_actual,2) + phase_shift_rad
                                            ))
                                    );
                            }
                        }
                    }
                } //end of function

                void compute_spoofing_signals(){
                    
                    //lock the mutex so that the attacking thread isn't trying to read paramter estimations while updating them at the same time
                    std::unique_lock<std::mutex> victim_parameters_lock(victim_parameters_mutex, std::defer_lock);

                    //lock the mutex
                    victim_parameters_lock.lock();

                    //reset the attack signal buffers
                    configure_attack_signal_buffers();

                    //mark the start time
                    auto start = high_resolution_clock::now();

                    //determine the number of spoofed signals
                    num_spoofing_signals = FN_spoof_enable ? 
                        current_FP_spoofing_positions_m.size() + 1 //if FN spoof is enabled
                        : current_FP_spoofing_positions_m.size(); //if FN spoof is not enabled

                    //compute the FP spoofing signals
                    if(FP_spoof_enable)
                    {
                        double additional_power_scaling_FP = 
                                                    FN_spoof_enable ? 
                                                    0.5 * (1/static_cast<double>(num_spoofing_signals - 1)) //if FN spoof enabled
                                                    : (1/static_cast<double>(num_spoofing_signals)); //if FN spoof not enabled
                        
                        //compute the FP spoofing signals
                        for (size_t i = 0; i < current_FP_spoofing_positions_m.size(); i++)
                        {
                            compute_spoof_signal(
                                current_FP_spoofing_positions_m[i],
                                current_FP_spoofing_velocities_m_s[i],
                                false, //similar slope attack not enabled
                                false, //similar velocity attack not enabled
                                additional_power_scaling_FP, //apply additional power scaling
                                false //don't reset attack signal buffers
                            );
                        }
                    }

                    //compute the FN spoofing signals
                    if(FN_spoof_enable)
                    {
                        double additional_power_scaling_FN = FP_spoof_enable ? 0.5 : 1.0; //0.5 if FP attack enabled, 1.0 if not
                        compute_spoof_signal(
                            current_FN_spoofing_pos_m,
                            current_FN_spoofing_vel_m_s,
                            sim_slope_attack_enable, //similar slope attack not enabled
                            sim_vel_attack_enable, //similar velocity attack not enabled
                            additional_power_scaling_FN, //apply additional power scaling
                            false //don't reset attack signal buffers
                        );
                    }

                    //mark the stop time
                    auto stop = high_resolution_clock::now();
                    auto duration = duration_cast<microseconds>(stop - start);
                    
                    //if debug is enabled print the time taken to compute the waveform
                    if(debug){
                        std::cout << "AttackingSubsystem::compute_spoofing_signals: Time taken to compute" << num_spoofing_signals <<   
                        " waveforms: " << duration.count() << " microseconds" << std::endl;
                    }

                    //unlock the mutex
                    victim_parameters_lock.unlock();
                }
                
                /**
                 * @brief Compute the spoofed signal for a single spoofed point
                 * 
                 * @param spoof_distance_m the distance to spoof at
                 * @param spoof_velocity_m_s the velocity to spoof at
                 * @param similar_slope_attack On True, computes signal of a similar slope attack. Defaults to False
                 * @param similar_velocity_attack On True, computes the signal of a similar velocity attack. Defaults to False.
                 * @param additional_power_scaling Additional power scaling to apply (if multiple signals are being transmitted). Defaults to 1.0 (no additional power scaling)
                 * @param reset_attack_signal_buffers On True, resets the attack signal buffers to zero. Defaults to true
                 */
                void compute_spoof_signal(
                    double spoof_distance_m,
                    double spoof_velocity_m_s,
                    bool similar_slope_attack = false,
                    bool similar_velocity_attack = false,
                    double additional_power_scaling = 1.0,
                    bool reset_attack_signal_buffers = true
                ){

                    //if desired reset the attack signal buffers
                    if (reset_attack_signal_buffers)
                    {
                        configure_attack_signal_buffers();
                    }

                    //compute the requisite parameters
                    double spoof_slope_MHz_us = compute_spoof_slope_MHz_us(similar_slope_attack);
                    double spoof_time_delay_s = compute_spoof_time_delay_s(spoof_distance_m,similar_slope_attack,spoof_slope_MHz_us);
                    //double spoof_power_scaling = compute_spoof_power_scaling(spoof_distance_m,(similar_slope_attack || similar_velocity_attack));
                    //TODO: re-enable power scaling and refine the method further
                    double spoof_power_scaling = 0.5;
                    std::vector<double> phase_shifts(chirps_per_frame,0);
                    compute_spoof_chirp_phase_shifts_rad(phase_shifts,spoof_velocity_m_s,similar_velocity_attack);

                    compute_chirp_waveforms_multi_threaded(
                        spoof_slope_MHz_us,
                        spoof_time_delay_s,
                        phase_shifts,
                        spoof_power_scaling * additional_power_scaling,
                        8, //max number of threads
                        false //overwrite attack chirps buffer
                        );

                    
                }

                //TODO: Remove this function once performance has been verified for the multi-threaded version
                /**
                 * @brief DEPRECIATED FUNCTION
                 * 
                 * @param spoof_slope_MHz_us 
                 * @param spoof_time_delay_s 
                 * @param phase_shifts 
                 * @param spoof_power_scaling 
                 */
                void compute_chirp_waveforms(
                    double spoof_slope_MHz_us,
                    double spoof_time_delay_s,
                    std::vector<double> & phase_shifts,
                    double spoof_power_scaling)
                {
                    //compute the waveform for each chirp
                    for (size_t i = 0; i < chirps_per_frame; i++)
                    {
                        compute_FMCW_waveform(
                            attack_chirps_buffer.buffer[i],
                            spoof_slope_MHz_us,
                            spoof_time_delay_s,
                            phase_shifts[i],
                            spoof_power_scaling,
                            true //overwite existing buffer
                        );
                    }
                }

                /**
                 * @brief Compute the chirps for a FMCW waveform and save them in the attack_chirps_buffer
                 * 
                 * @param spoof_slope_MHz_us The chirp slope in MHz/us
                 * @param spoof_time_delay_s The time delay in seconds
                 * @param phase_shifts A vector of phase shifts in radians corresponding to each chirp
                 * @param spoof_power_scaling Power scaling to be applied to the generated waveform. Defaults to 1.0 (no scaling)
                 * @param max_threads The maximum number of threads to use when computing the waveform. Defaults to 8.
                 * @param overwrite_attack_chirps_buffer On True, will overwrite the attack chirp buffer with the newly computed FMCW waveform. On False, will add the newly computed FMCW waveform to the existing signal in the attack chirp buffer. Defaults to true.
                 */
                void compute_chirp_waveforms_multi_threaded(
                    double spoof_slope_MHz_us,
                    double spoof_time_delay_s,
                    std::vector<double> & phase_shifts,
                    double spoof_power_scaling = 1.0,
                    size_t max_threads = 8,
                    bool overwrite_attack_chirps_buffer = true)
                {
                    std::vector<std::thread> threads;
                    const size_t num_threads = std::min(chirps_per_frame, max_threads);
                    for (size_t i = 0; i < num_threads; i++)
                    {
                        threads.emplace_back([&, i](){
                            const size_t start = i * chirps_per_frame / num_threads;
                            const size_t end = (i + 1) * chirps_per_frame / num_threads;
                            for (size_t j = start; j < end; j++)
                            {
                                compute_FMCW_waveform(
                                    attack_chirps_buffer.buffer[j],
                                    spoof_slope_MHz_us,
                                    spoof_time_delay_s,
                                    phase_shifts[j],
                                    spoof_power_scaling,
                                    overwrite_attack_chirps_buffer
                                );
                            }
                        });
                    }

                    for (auto& thread : threads)
                    {
                        thread.join();
                    }

                    return;
                }

                /**
                 * @brief Reconfigure the attack signal buffers to be the correct dimmensions for transmission on the USRP and computing the waveforms
                 * 
                 */
                void configure_attack_signal_buffers(){
                    
                    //configure the buffer that will be used by the USRP to transmit attack signals
                    USRP_attack_signal_buffer.configure_fmcw_buffer(
                        samples_per_buffer,
                        num_samples_per_chirp,
                        chirps_per_frame,
                        false);
                    
                    //configure the buffer that will be used when computing the attack signals for each chirp
                    attack_chirps_buffer.reconfigure(
                        chirps_per_frame,
                        num_samples_per_chirp);
                }

                /**
                 * @brief Compute the time delay required to spoof an object at a specific location. A slight offset is applied in the case of a similar slope attack
                 * 
                 * @param spoof_distance_m the spoof distance that the object is desired to appear at from the perspective of the victim
                 * @param similar_slope_attack On true, applies an additional offset for the case of a similar slope attack
                 * @param slope_MHz_us - when a similar slope attack is used, this parameter is used to account for the slope of the attacking signal
                 * @return double the time delay required to spoof in seconds
                 */
                double compute_spoof_time_delay_s(double spoof_distance_m, bool similar_slope_attack = false, double slope_MHz_us = 0.0){
                    
                    //compute the time delay for the spoof
                    double time_delay_s = (2 * spoof_distance_m - current_victim_pos_m)/c_m_s;

                    //compute an additional time delay if a similar_slope_attack is enabled
                    if (similar_slope_attack)
                    {
                        double additional_time_delay_s;
                        if (FMCW_sampling_rate_Hz >= (50 * 1e6))
                        {
                            additional_time_delay_s = (0.4 * FMCW_sampling_rate_Hz * 1e-6 *
                                (1/estimated_frequency_slope_MHz_us - 1/slope_MHz_us)) * 1e-6;
                        }
                        else
                        {
                            additional_time_delay_s = (0.6 * FMCW_sampling_rate_Hz * 1e-6 *
                                (1/estimated_frequency_slope_MHz_us - 1/slope_MHz_us)) * 1e-6; //was previously 0.4 * FMCW
                        }
                        time_delay_s += additional_time_delay_s;
                    }

                    return time_delay_s;
                }

                /**
                 * @brief Compute the phase shift (in radians) required to spoof an object at a relative velocity. Also takes into acount the relative velocity of the victim with respect to the attacker
                 * 
                 * @param phase_shifts A reference to a vector where the computed phase shifts will be stored
                 * @param spoof_velocity_m_s The desired velocity to spoof at
                 * @param similar_velocity_attack on True, computes slightly different phase shifts to cause a similar velocity attack
                 */
                void compute_spoof_chirp_phase_shifts_rad(
                    std::vector<double> & phase_shifts,
                    double spoof_velocity_m_s, 
                    bool similar_velocity_attack = false){

                    if(! similar_velocity_attack){
                        //compute the phase shift at each chirp
                        double phase_shift_per_chirp = 
                            4 * M_PI * (spoof_velocity_m_s - (current_victim_vel_m_s/2)) *
                             estimated_chirp_cycle_time_us * 1e-6 / lambda_m;

                        for (size_t i = 0; i < chirps_per_frame; i++)
                        {
                            phase_shifts[i] = phase_shift_per_chirp * static_cast<double>(i);
                        }
                    }
                    else //initialize the phase shifts to be for the similar velocity attack
                    {
                        compute_spoof_chirp_phase_shifts_rad_sim_vel(phase_shifts, spoof_velocity_m_s);
                    }
                    return;
                }

                /**
                 * @brief Compute the phase shift (in radians) required to spoof an object at a relative velocity while performing the similar velocity attack. Also takes into acount the relative velocity of the victim with respect to the attacker
                 * 
                 * @param phase_shifts A reference to a vector where the computed phase shifts will be stored
                 * @param spoof_velocity_m_s The desired velocity to spoof at
                 * @return std::vector<double> a vector containing the phase shift (in radians) to be applied to each chirp
                 */
                void compute_spoof_chirp_phase_shifts_rad_sim_vel(
                    std::vector<double> & phase_shifts, 
                    double spoof_velocity_m_s){
                    
                    double frac_bins_to_target = 0.20;
                    //ensure that we target at least 3 velocity bins
                    double num_bins_to_target = (0.2 * static_cast<double>(chirps_per_frame) > 3) ? 0.2 * static_cast<double>(chirps_per_frame)  : 3;

                    //compute the velocity resolution
                    double v_res = lambda_m / (2 * static_cast<double>(chirps_per_frame) * estimated_chirp_cycle_time_us * 1e-6);

                    //compute the velocity spread based on the number of bins we seek to target
                    double v_spread = v_res * num_bins_to_target;

                    //compute the initial velocity value
                    double v_0 = spoof_velocity_m_s - (v_spread/2);

                    //compute the delta velocity value
                    double delta_v = v_spread/ static_cast<double>(chirps_per_frame);

                    //set the first phase shift to be zero
                    phase_shifts[0] = 0;

                    //compute the remaining phase shifts
                    for (size_t i = 1; i < chirps_per_frame; i++)
                    {
                        phase_shifts[i] = (4 * M_PI * ((v_0 + static_cast<double>(i) * delta_v) - current_victim_vel_m_s / 2) * 
                            estimated_chirp_cycle_time_us * 1e-6 / lambda_m) + phase_shifts[i-1];
                    }

                    return;
                }

                /**
                 * @brief Compute the chirp slope in MHz/us for the generated attacking signal
                 * 
                 * @param similar_slope_attack - on True, slighly modifies the spoofing slope to enable a similar slope attack. On false, just returns the estimated frequency slope
                 * @return double - the spooifng frequency slope in MHz/us
                 */
                double compute_spoof_slope_MHz_us(bool similar_slope_attack = false){
                    
                    //slightly modify the spoofing slope if similar slope attack is enabled
                    if (similar_slope_attack)
                    {
                        if (FMCW_sampling_rate_Hz >= (50 * 1e6))
                        {
                            return estimated_frequency_slope_MHz_us + (estimated_frequency_slope_MHz_us * 0.015);
                        }
                        else
                        {   
                            //TODO: add this parameter to the JSON configuration
                            double range_span_m = 75;
                            double delta_IF_MHz = 2 * estimated_frequency_slope_MHz_us * 1e6 * range_span_m / c_m_s;
                            double additional_slope_MHz_us = delta_IF_MHz / estimated_chirp_cycle_time_us;
                            return estimated_frequency_slope_MHz_us + additional_slope_MHz_us;

                            //previous computation
                            //return estimated_frequency_slope_MHz_us + (estimated_frequency_slope_MHz_us * 5);   
                        }
                        
                    }
                    else
                    {
                        return estimated_frequency_slope_MHz_us;
                    }
                    
                }

                /**
                 * @brief Compute the amount to scale the spoofed signal's power by to replicate realistic loss due to propagation for spoofing objects
                 * 
                 * @param spoof_distance_m 
                 * @param FN_attack 
                 * @return double 
                 */
                double compute_spoof_power_scaling(double spoof_distance_m, bool FN_attack = false){
                    
                    //if this is a false negative attack, do not cause any backoff
                    if (FN_attack)
                    {
                        return 1.0;
                    }
                    else if (spoof_distance_m < current_victim_pos_m)
                    {
                        //attack will transmit at full power if attacker is further away than desired spoofing distance
                        return 1.0;
                    }
                    else
                    {
                        double loss_attacker = compute_one_way_propagation_loss(current_victim_pos_m);
                        double loss_spoofing = compute_one_way_propagation_loss(spoof_distance_m);
                        return loss_spoofing/loss_attacker;
                    }
                    
                }

                double compute_one_way_propagation_loss(double propagation_distance_m){
                    return 1.0 / (4 * M_PI * std::pow(propagation_distance_m,2));
                }


        };
    }
#endif