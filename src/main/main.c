#include <limits.h>

#include "DSP28x_Project.h"

#include "f2802x_common/include/clk.h"
#include "f2802x_common/include/flash.h"
#include "f2802x_common/include/gpio.h"
#include "f2802x_common/include/pie.h"
#include "f2802x_common/include/pll.h"
#include "f2802x_common/include/spi.h"
#include "f2802x_common/include/timer.h"
#include "f2802x_common/include/wdog.h"

#include "comm.h"
#include "scheduler.h"

#pragma CODE_SECTION( commandReceive, "ramfuncs" );
#pragma CODE_SECTION( toggleLedsInterrupt, "ramfuncs" );

static void commandReceive( void );
static void handlesInit( void );
static void gpioInit( void );
static void spiInit( void );
static void interruptInit( void );
static interrupt void toggleLedsInterrupt( void );

CLK_Handle myClk;
CPU_Handle myCpu;
#ifdef _FLASH
FLASH_Handle myFlash;
#endif
GPIO_Handle myGpio;
PIE_Handle myPie;
PLL_Handle myPll;
SPI_Handle mySpi;
TIMER_Handle myTimer;
WDOG_Handle myWDog;

void main( void ) {
    handlesInit();
    gpioInit();
    spiInit();
    interruptInit();
    commandReceive();
}

static void commandReceive( void ) {
    uint16_t command[ sizeof( Command_t ) ];
    uint8_t readData;
    size_t i = sizeof( uint8_t ), j;

    for( ;; ) {
        for( i = 0; i < sizeof( Command_t ); i++ ) {
            for( j = 0; j < 2; j++ ) {
                command[i] >>= 8;
                while( SPI_getRxFifoStatus( mySpi ) == SPI_FifoStatus_Empty ) {}
                readData = SPI_read( mySpi );
                SPI_write8( mySpi, readData );
                command[i] |= ( readData << 8 );
            }
        }
        while( SPI_getRxFifoStatus( mySpi ) == SPI_FifoStatus_Empty ) {}
        readData = SPI_read( mySpi );
        SPI_write8( mySpi, readData );
        applyCommand( ( Command_t* )( command ) );
    }
}

static void handlesInit( void ) {
    myClk = CLK_init( ( void * )CLK_BASE_ADDR, sizeof( CLK_Obj ) );
    myCpu = CPU_init( ( void * )NULL, sizeof( CPU_Obj ) );
#ifdef _FLASH
    myFlash = FLASH_init( (void * )FLASH_BASE_ADDR, sizeof( FLASH_Obj ) );
#endif
    myGpio = GPIO_init( ( void * )GPIO_BASE_ADDR, sizeof( GPIO_Obj ) );
    myPie = PIE_init( ( void * )PIE_BASE_ADDR, sizeof( PIE_Obj ) );
    myPll = PLL_init( ( void * )PLL_BASE_ADDR, sizeof( PLL_Obj ) );
    mySpi = SPI_init( ( void * )SPIA_BASE_ADDR, sizeof( SPI_Obj ) );
    myTimer = TIMER_init( ( void * )TIMER0_BASE_ADDR, sizeof( TIMER_Obj ));
    myWDog = WDOG_init( ( void * )WDOG_BASE_ADDR, sizeof( WDOG_Obj ));

    // Perform basic system initialization
    WDOG_disable( myWDog );
    CLK_enableAdcClock( myClk );
    ( *Device_cal )();

    // Enable SPI-A Clock
    CLK_enableSpiaClock( myClk );

    //Select the internal oscillator 1 as the clock source
    CLK_setOscSrc( myClk, CLK_OscSrc_Internal );

    // Setup the PLL for x12 /2 which will yield 60Mhz = 10Mhz * 12 / 2
    PLL_setup( myPll, PLL_Multiplier_12, PLL_DivideSelect_ClkIn_by_2 );

    // Disable the PIE and all interrupts
    PIE_disable( myPie );
    PIE_disableAllInts( myPie );
    CPU_disableGlobalInts( myCpu );
    CPU_clearIntFlags( myCpu );

    // If running from flash copy RAM only functions to RAM
#ifdef _FLASH
    memcpy( &RamfuncsRunStart, &RamfuncsLoadStart, ( size_t )&RamfuncsLoadSize );
#endif

    // Setup a debug vector table and enable the PIE
#ifdef _DEBUG
    PIE_setDebugIntVectorTable( myPie );
#endif
    PIE_enable( myPie );
}

static void gpioInit( void ) {
    GPIO_setMode( myGpio, GPIO_Number_0, GPIO_0_Mode_GeneralPurpose );
    GPIO_setMode( myGpio, GPIO_Number_1, GPIO_0_Mode_GeneralPurpose );
    GPIO_setMode( myGpio, GPIO_Number_2, GPIO_0_Mode_GeneralPurpose );
    GPIO_setMode( myGpio, GPIO_Number_3, GPIO_0_Mode_GeneralPurpose );
    GPIO_setDirection( myGpio, GPIO_Number_0, GPIO_Direction_Output );
    GPIO_setDirection( myGpio, GPIO_Number_1, GPIO_Direction_Output );
    GPIO_setDirection( myGpio, GPIO_Number_2, GPIO_Direction_Output );
    GPIO_setDirection( myGpio, GPIO_Number_3, GPIO_Direction_Output );
    GPIO_setLow( myGpio, GPIO_Number_0 );
    GPIO_setHigh( myGpio, GPIO_Number_1 );
    GPIO_setHigh( myGpio, GPIO_Number_2 );
    GPIO_setLow( myGpio, GPIO_Number_3 );

    GPIO_setPullUp( myGpio, GPIO_Number_16, GPIO_PullUp_Enable );
    GPIO_setPullUp( myGpio, GPIO_Number_17, GPIO_PullUp_Enable );
    GPIO_setPullUp( myGpio, GPIO_Number_18, GPIO_PullUp_Enable );
    GPIO_setPullUp( myGpio, GPIO_Number_19, GPIO_PullUp_Enable );
    GPIO_setQualification( myGpio, GPIO_Number_16, GPIO_Qual_ASync );
    GPIO_setQualification( myGpio, GPIO_Number_17, GPIO_Qual_ASync );
    GPIO_setQualification( myGpio, GPIO_Number_18, GPIO_Qual_ASync );
    GPIO_setQualification( myGpio, GPIO_Number_19, GPIO_Qual_ASync );
    GPIO_setMode( myGpio, GPIO_Number_16, GPIO_16_Mode_SPISIMOA );
    GPIO_setMode( myGpio, GPIO_Number_17, GPIO_17_Mode_SPISOMIA );
    GPIO_setMode( myGpio, GPIO_Number_18, GPIO_18_Mode_SPICLKA );
    GPIO_setMode( myGpio, GPIO_Number_19, GPIO_19_Mode_SPISTEA_NOT );
}

static void spiInit( void ) {
    SPI_reset( mySpi );

    SPI_setCharLength( mySpi, SPI_CharLength_8_Bits );
    SPI_setClkPhase( mySpi, SPI_ClkPhase_Normal );
    SPI_setClkPolarity( mySpi, SPI_ClkPolarity_OutputFallingEdge_InputRisingEdge );
    SPI_setMode( mySpi, SPI_Mode_Slave );
    SPI_enableTx( mySpi );
    SPI_setTxDelay( mySpi, 0 );

    // Initialize SPI FIFO registers
    SPI_enableFifoEnh( mySpi );
    SPI_enableChannels( mySpi );
    SPI_resetTxFifo( mySpi );
    SPI_resetRxFifo( mySpi );
    SPI_enableTxFifo( mySpi );
    SPI_enableRxFifo( mySpi );

    // Set so breakpoints don't disturb transmission
    SPI_setPriority( mySpi, SPI_Priority_Immediate );

    SPI_enable( mySpi );
}

static void interruptInit( void ) {
    // Register interrupt handlers in the PIE vector table
    PIE_registerPieIntHandler( myPie, PIE_GroupNumber_1, PIE_SubGroupNumber_7, ( intVec_t )&toggleLedsInterrupt );

    // Configure CPU-Timer 0 to interrupt every 500 milliseconds:
    // 60MHz CPU Freq, 50 millisecond Period ( in uSeconds )
    //    ConfigCpuTimer( &CpuTimer0, 60, 500000 );
    TIMER_stop( myTimer );
    TIMER_setPeriod( myTimer, 50 * 500000 );
    TIMER_setPreScaler( myTimer, 0 );
    TIMER_reload( myTimer );
    TIMER_setEmulationMode( myTimer, TIMER_EmulationMode_StopAfterNextDecrement );
    TIMER_enableInt( myTimer );
    TIMER_start( myTimer );

    CPU_enableInt( myCpu, CPU_IntNumber_1 );
    PIE_enableTimer0Int( myPie );

    // Enable global Interrupts and higher priority real-time debug events
    CPU_enableGlobalInts( myCpu );
    CPU_enableDebugInt( myCpu );
}

static interrupt void toggleLedsInterrupt( void ) {
    GPIO_toggle( myGpio, GPIO_Number_0 );
    GPIO_toggle( myGpio, GPIO_Number_1 );
    GPIO_toggle( myGpio, GPIO_Number_2 );
    GPIO_toggle( myGpio, GPIO_Number_3 );

    // Acknowledge this interrupt to receive more interrupts from group 1
    PIE_clearInt( myPie, PIE_GroupNumber_1 );
}
