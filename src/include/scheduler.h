typedef struct MotorMovement MotorMovement;

enum MotorStatus {
    Idle = 0,
    Accelerating = 1,
    Deaccelerating = 2,
    ConstantSpeed = 3
};

struct MotorMovement {
    enum MotorStatus motorStatus;
    int32_t steps;
    int16_t fractionalStep;
    int8_t maxSpeed;
    int8_t speed;
    int16_t fractionalSpeed;
    int8_t acceleration;
    int16_t fractionalAcceleration;
};

int schedulerInit( void );
