
#include <libovr_nsb/OVR.h>

static Device *dev = NULL;
static float quat[4];

/////////////////////////////////////////////////////////////////////////////////////
// Continuous sample/update thread code
// select() chosen for portability
/////////////////////////////////////////////////////////////////////////////////////
void *threadFunc( void *data )
{
    Device *localDev = (Device *)data;

    while( localDev->runSampleThread )
    {
        // Try to sample the device for 1ms
        waitSampleDevice(localDev, 1000);

        // Send a keepalive - this is too often.  Need to only send on keepalive interval
        sendSensorKeepAlive(localDev);
    }
    return 0;
}

void init_device()
{
    dev = openRift(0,0);

    if( !dev )
    {
        printf("Could not locate Rift\n");
        printf("Be sure you have read/write permission to the proper /dev/hidrawX device\n");
    }

    setKeepAliveInterval(dev, 1000);

    pthread_t f1_thread;
    dev->runSampleThread = TRUE;
    pthread_create(&f1_thread,NULL,threadFunc,dev);
}

float *get_quatanion()
{
	int i;
	for(i=0;i<4;i++)
	{
		quat[i] = dev->Q[i];
	}
	return quat;
}
