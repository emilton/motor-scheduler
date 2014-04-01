#define NUM_MOTORS 4
#define MAX_COMMANDS 16

enum CommandType {
    NoOp = 0,
    ConstantSpeed = 1,
    Accelerating  = 2,
    Configuration = 3,
    EmergencyStop = 4
};

typedef struct ConstantSpeed_t ConstantSpeed_t;
typedef struct Accelerating_t  Accelerating_t;
typedef struct Configuration_t Configuration_t;

typedef struct Command_t Command_t;

struct Accelerating_t {
    int32_t steps[NUM_MOTORS];
    int32_t accelerations[NUM_MOTORS];
};

struct ConstantSpeed_t {
    int32_t steps[NUM_MOTORS];
    int32_t speeds[NUM_MOTORS];
};

struct Configuration_t {
    int32_t properties[NUM_MOTORS];
    int32_t initialize[NUM_MOTORS];
};

struct Command_t {
    char commandType;
    Command_t* nextCommand;
    union {
    	Configuration_t configuration;
        Accelerating_t  accelerating;
        ConstantSpeed_t constantSpeed;
    } command;
};


