#define NUM_MOTORS 3

enum CommandType {
    NoOp = 0,
    ConstantSpeed = 1,
    Accelerating = 2
};

typedef struct Accelerating_t Accelerating_t;
typedef struct ConstantSpeed_t ConstantSpeed_t;
typedef struct Command_t Command_t;

struct Accelerating_t {
    int32_t steps[NUM_MOTORS];
    int32_t accelerations[NUM_MOTORS];
};

struct ConstantSpeed_t {
    int32_t steps[NUM_MOTORS];
    int32_t speeds[NUM_MOTORS];
};

struct Command_t {
    char commandType;
    union {
        Accelerating_t accelerating;
        ConstantSpeed_t constantSpeed;
    } command;
};


