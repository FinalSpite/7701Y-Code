using namespace vex;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void );
void run( void );
void turn_to_angle( float );
int rc_auto_loop_function_Controller1();
int BrainScreenUpdate();
int odomTrack();
void moveTo( double, double );
void drawLogo( void );
void smoothDrive( int , int );
void jitter(int);