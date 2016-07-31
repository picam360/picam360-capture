
#include <libovr_nsb/OVR.h>

static Device *dev = NULL;
static float quat[4];

void init_device()
{
    dev = openRift(0,0);

    if( !dev )
    {
        printf("Could not locate Rift\n");
        printf("Be sure you have read/write permission to the proper /dev/hidrawX device\n");
    }

    sendSensorKeepAlive(dev);
}

float *get_quatanion()
{
	int i;
    waitSampleDevice(dev, 1000);
    sendSensorKeepAlive(dev);
    printf("\tQ:%+-10g %+-10g %+-10g %+-10g\n", dev->Q[0], dev->Q[1], dev->Q[2], dev->Q[3] );

	for(i=0;i<4;i++)
	{
		quat[i] = dev->Q[i];
	}
	return quat;
}
