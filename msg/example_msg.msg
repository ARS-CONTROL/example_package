## Serialization || C++ || Python3 

# unsigned 8-bit int || uint8_t || bool
bool boolean_value__bool

# signed 8-bit / 16-bit / 32-bit int || int8_t / int16_t / int32_t || int
int8 int8_value__int
int16 int16_value__int
int32 int32_value__int

# signed 64-bit int || int64_t || long
int64 int64_value__long_int

# unsigned 8-bit / 16-bit / 32-bit int || uint8_t / uint16_t / uint32_t || int
uint8 uint8_value__unsigned_int
uint8 uint16_value__unsigned_int
uint8 uint32_value__unsigned_int

# unsigned 64-bit int || uint64_t || long
uint8 uint64_value__unsigned_long_int

# 32-bit IEEE float || float || float
float32 float32_value__float
	
# 64-bit IEEE float || double || float
float64 float64_value__double

# ascii string || std::string || str
string string_value__std_string

# secs/nsecs unsigned 32-bit ints || ros::Time || rospy.Time
time ros_time_value	

# secs/nsecs signed 32-bit ints || ros::Duration || rospy.Duration
duration ros_duration_value


##################################################
##                   Arrays                     ##
##################################################

# variable-lenght || std::vector<Type> || list of Types
int32[] array_value__std_vector_int
float64[] array_value__std_vector_double
string[] array_value__std_vector_string


##################################################
##                  Costants                    ##
##################################################

# constanttype CONSTANTNAME=constantvalue
int32 CONSTANT1=0
string CONSTANT2="EXAMPLE"
