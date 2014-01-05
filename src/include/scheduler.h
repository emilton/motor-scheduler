typedef struct Movement Movement;

enum MotorSpeed {
    Idle = 0,
    Accelerating = 1,
    Deaccelerating = 2,
    Constant = 3
};

struct Movement {
    enum MotorSpeed motorSpeed;
    uint32_t steps;
    uint16_t fractionalStep;
};