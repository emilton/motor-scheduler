#define sign(x) ( x >= 0 )

typedef struct MotorMovement MotorMovement;

struct MotorMovement {
    int32_t steps;
    int32_t fractionalStep;
    int32_t speed;
    int32_t acceleration;
};

extern int schedulerInit( void );
extern int applyCommand( Command_t *command );
extern int updateMotors( void );
extern int moveMotor( int motorNumber );
extern int isHomed( int motorNumber );
extern int setDirection( int motorNumber, int forwardDirection );
