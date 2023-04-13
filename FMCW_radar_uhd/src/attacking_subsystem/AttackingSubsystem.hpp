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

                //mutex for frame start times to ensure thread safety
                std::mutex frame_start_times_mutex;
                std::mutex sensing_complete_mutex;

            public:
                size_t attack_start_frame;
            private:

                //RF parameters for the attackers and support parameters
                const double c = 2.99792458e8;
                double lambda_m; 
                double FMCW_sampling_rate_Hz;
                double FMCW_sampling_period_s;
                
                //estimated victim parameters used to compute victim waveform
                //TODO: when computing the attacking waveforms, uncomment the other variables
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

            private: //make attack signals private as it should only be accessed by this class
                //variables for generating attack signal
                //TODO: add these variables in

                //attack signal buffer
                double samples_per_buffer;
                std::string attack_signal_file;
                Buffer_2D<std::complex<data_type>> attack_signal_buffer;

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
                                                                    attack_start_frame(rhs.attack_start_frame),
                                                                    c(rhs.c),
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
                                                                    samples_per_buffer(rhs.samples_per_buffer),
                                                                    attack_signal_file(rhs.attack_signal_file),
                                                                    attack_signal_buffer(rhs.attack_signal_buffer)
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
                        //both of the mutexes are not copyable
                        attack_start_frame = rhs.attack_start_frame;
                        //c is a constant and doesn't need to be set
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
                        samples_per_buffer = rhs.samples_per_buffer;
                        attack_signal_file = rhs.attack_signal_file;
                        attack_signal_buffer = rhs.attack_signal_buffer;
                    }

                    return *this;
                }

                ~AttackingSubsystem(){}

            public: //public initialization function
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
                        //initialize key attacker parameters regardless
                        initialize_attack_subsystem_parameters();
                        //initialize victim parameter estimation capability regardless of whether or not attacker is enabled
                        init_estimated_parameter_values();
                        if (enabled)
                        {
                            init_attack_signal_buffer();
                            init_frame_start_times_buffer();
                        }
                        
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

                    //tx file name
                    if(config["AttackSubsystemSettings"]["tx_file_name"].is_null()){
                        std::cerr << "AttackSubsystem::check_config: no tx_file_name in JSON" <<std::endl;
                        config_good = false;
                    }

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

                    return config_good;
                }

                /**
                 * @brief Initialize attack subsystem parameters
                 * 
                 */
                void initialize_attack_subsystem_parameters(){
                    
                    //attack signal file path
                    attack_signal_file = config["AttackSubsystemSettings"]["tx_file_name"].get<std::string>();
                    
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
                    lambda_m = c / config["USRPSettings"]["Multi-USRP"]["center_freq"].get<double>();
                    
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
                }
                
                /**
                 * @brief load the pre-computed attack signal into the attack signal buffer
                 * 
                 */
                void init_attack_signal_buffer(){

                    //set the buffer path
                    attack_signal_buffer.set_read_file(attack_signal_file,true);

                    //load the samples into the buffer
                    attack_signal_buffer.import_from_file(samples_per_buffer);
                }


            public: //the following functions are public and meant to be called by other classes
                
                void compute_calculated_values(
                    double chirp_cycle_time_us,
                    double frequency_slope_MHz_us,
                    double frame_periodicity_ms,
                    size_t num_chirps
                ){
                    

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

                    //TODO: add code to compute attack signals in real time

                    //initialize the victim waveform buffer to be the correct size
                    victim_waveform.set_buffer_size(num_samples_per_chirp);

                    //compute the victim waveform (with no time delay, phase shift, or power scaling)
                    compute_FMCW_waveform(
                        victim_waveform.buffer,
                        estimated_frequency_slope_MHz_us,
                        0,0,1.0);

                    //set flag noting that a victim waveform has been loaded
                    victim_waveform_loaded = true;
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
                            attacker_usrp_handler -> stream_frames_tx_only(frame_start_time, & attack_signal_buffer);
                            
                        }
                        else{
                            frame_start_times_lock.unlock();
                            //std::this_thread::sleep_for(std::chrono::milliseconds(int64_t(1)));
                        }
                        
                    }
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
                    initialize_attack_subsystem_parameters();
                    if (enabled)
                    {
                        init_attack_signal_buffer();
                    }
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
                 * @param phase_shift Any phase shift (in radians) to apply (for spoofing velocity)
                 * @param power_scaling Power scaling for the FMCW waveform (for adjusting with respect to range)
                 */
                void compute_FMCW_waveform(
                    std::vector<std::complex<data_type>> & result_location,
                    double slope_MHz_us,
                    double time_delay_s,
                    double phase_shift,
                    data_type power_scaling){
                    
                    //variable to store the time to compute each sample at (factoring in the delay)
                    double t_actual = 0;

                    //compute the FMCW waveform for each time step in the t_sec vector
                    for (size_t i = 0; i < num_samples_per_chirp; i++)
                    {
                        t_actual = t_sec[i] - time_delay_s;
                        if (t_actual < 0) 
                        {
                            //TODO: see if this actually sets the value in the desired array, if not, use (*result_location)[i] 
                            //and pass the result_location as a pointer instead
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
                                            M_PI * estimated_frequency_slope_MHz_us * 1e-12 *
                                            std::pow(t_actual,2) + phase_shift
                                        ))
                                );
                        }
                        
                    }
                    
                }
        };
    }
#endif