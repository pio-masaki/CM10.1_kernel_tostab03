/*
** =========================================================================
** File:
**     ImmVibeSPI.c
**
** Description: 
**     Device-dependent functions called by Immersion TSP API
**     to control PWM duty cycle, amp enable/disable, save IVT file, etc...
**
** Portions Copyright (c) 2008-2009 Immersion Corporation. All Rights Reserved. 
**
** This file contains Original Code and/or Modifications of Original Code 
** as defined in and that are subject to the GNU Public License v2 - 
** (the 'License'). You may not use this file except in compliance with the 
** License. You should have received a copy of the GNU General Public License 
** along with this program; if not, write to the Free Software Foundation, Inc.,
** 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA or contact 
** TouchSenseSales@immersion.com.
**
** The Original Code and all software distributed under the License are 
** distributed on an 'AS IS' basis, WITHOUT WARRANTY OF ANY KIND, EITHER 
** EXPRESS OR IMPLIED, AND IMMERSION HEREBY DISCLAIMS ALL SUCH WARRANTIES, 
** INCLUDING WITHOUT LIMITATION, ANY WARRANTIES OF MERCHANTABILITY, FITNESS 
** FOR A PARTICULAR PURPOSE, QUIET ENJOYMENT OR NON-INFRINGEMENT. Please see 
** the License for the specific language governing rights and limitations 
** under the License.
** =========================================================================
*/

#ifdef 	IMMVIBESPIAPI
#undef 	IMMVIBESPIAPI
#endif
#define 	IMMVIBESPIAPI static

#include <../kernel/arch/arm/mach-tegra/nv/include/nvcommon.h>
#include <../kernel/arch/arm/mach-tegra/gpio-names.h>
#include <linux/module.h> 
#include <linux/kernel.h>
#include <linux/init.h> 
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/pwm.h>
#include <linux/delay.h>
#include <linux/kthread.h> 
#include <linux/interrupt.h>
#include <linux/fb.h>


/* This SPI supports only one actuator. */
#define 	NUM_ACTUATORS 	1

#define amp_shutdown_pin TEGRA_GPIO_PD7
#define NvOdmPwmMode_Disable 1
#define NvOdmPwmMode_Enable  2

#define dock_in_pin TEGRA_GPIO_PD6
#define ENABLE_HAPTIC		0
#define DISABLE_HAPTIC		1

struct pwm_vibrator 
{
	struct pwm_device *pwm;
	unsigned long duty_cycle;
	unsigned long pwmFreq_set;
	unsigned long	time_ms;
};

enum 
{
	ON_DOCK_IN=0,
	NO_DOCK_IN
};

NvU32	g_requestedPeriod = 43000; //23.44k
static bool g_bAmpEnabled = false;
static struct pwm_vibrator *haptic_vibrator;
static int dock_in_state=0, haptic_state=ENABLE_HAPTIC;


/* static int 	vib_set_power_rail( NvU32 vdd_id, NvBool is_enable ) */
/* { */
/* 	return 0; */
/* } */

static void vibrator_pwm_config(int dock_in_para)
{
	if (IS_ERR(haptic_vibrator->pwm)) 
	{
		printk("haptic_vibrator->pwm error\n");
		return ;
	}
	
	gpio_direction_output(amp_shutdown_pin, dock_in_para);
	return;
}

static void update_nvdock_in_status(void)
{
	dock_in_state=gpio_get_value(dock_in_pin);
	
	switch(dock_in_state)
	{
		case ON_DOCK_IN:
			vibrator_pwm_config(ON_DOCK_IN);
			haptic_state=DISABLE_HAPTIC;
			break;
		case NO_DOCK_IN:
			haptic_state=ENABLE_HAPTIC;
			break;
		default:
			haptic_state=ENABLE_HAPTIC;
			break;
	}
	return;
}

static irqreturn_t nvdock_in_irq(int irq, void *data)
{	
	msleep(40);
	update_nvdock_in_status();
	
    return IRQ_HANDLED;
}

static int vib_init(void)
{
	int 	err, ret=0;

	#if 1 /* program dock_in pin to the interrupt pin */
	tegra_gpio_enable(dock_in_pin);
	gpio_request(dock_in_pin, "dock_det_in");
    gpio_direction_input(dock_in_pin);
	ret = request_threaded_irq(TEGRA_GPIO_TO_IRQ(dock_in_pin), NULL, nvdock_in_irq, IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING, "nvdock_in", NULL);
    if (!ret) 
	{
        enable_irq_wake(TEGRA_GPIO_TO_IRQ(dock_in_pin));
    }
    else
	{
		printk("nvdock_in request irq error\n");
		return err;
	}
	#endif

	haptic_vibrator = kzalloc(sizeof(*haptic_vibrator), GFP_KERNEL);
	if (!haptic_vibrator)
	{
		err = -1;
		printk( "[ImmVibeSPI][%s] fail to kzalloc\n", __func__ );
		return err;
	}

	//gpio_request(TEGRA_GPIO_PU4, "PWM_MAG_SW");
	//tegra_gpio_enable(TEGRA_GPIO_PU4);
	
	haptic_vibrator->pwm = pwm_request(1, "vibrator");

	if (IS_ERR(haptic_vibrator->pwm)) 
	{
		err = -1;
		printk("immVibe Failed to request pwm vibrator device\n");
		return err;
	}
	pwm_disable(haptic_vibrator->pwm);
	
	tegra_gpio_enable(amp_shutdown_pin);
	err = gpio_request(amp_shutdown_pin, "AMP_SHDN_CTRL");
	if(err)
	{
		err = -1;
		printk("immVibe Failed to request gpio AMP_SHDN_CTRL\n");
		return err;
	}
	gpio_direction_output(amp_shutdown_pin, 0);
	
	update_nvdock_in_status();

	return 0;

}


static void vib_enable( NvBool on )
{
	gpio_direction_output(amp_shutdown_pin, on);
}

// mode [1] : disable
// mode [2] : enable
static void vib_generatePWM( int mode )
{
	NvU32	DutyCycle;
	
	DbgOut(( "[ImmVibeSPI] : vib_generatePWM start.. mode[%d]\n", mode ));	
	
	if(mode==NvOdmPwmMode_Enable)
	{
		DutyCycle = g_requestedPeriod >> 1; // 50% duty
		
		pwm_enable(haptic_vibrator->pwm);
		pwm_config(haptic_vibrator->pwm, DutyCycle, g_requestedPeriod);
		
	}
	else
	{
		pwm_disable(haptic_vibrator->pwm);
	}
	
	return;
}


/* Called to disable amp (disable output force) */
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_AmpDisable( VibeUInt8 nActuatorIndex )
{
	if ( g_bAmpEnabled ) {
		
		DbgOut(( "[ImmVibeSPI] : ImmVibeSPI_ForceOut_AmpDisable\n"));
		
    	g_bAmpEnabled = false;

		// Disable GPIO(enable pin)
		vib_enable( NV_FALSE );

		// Disable PWM CLK
		vib_generatePWM( NvOdmPwmMode_Disable );
	}

	return VIBE_S_SUCCESS;
}

/* Called to enable amp (enable output force) */
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_AmpEnable( VibeUInt8 nActuatorIndex )
{
	if(haptic_state==DISABLE_HAPTIC) // levi debug
	{
		g_bAmpEnabled = false;
		vib_enable( NV_FALSE );
		vib_generatePWM( NvOdmPwmMode_Disable );
		return VIBE_S_SUCCESS;
	}
	
 	if ( ! g_bAmpEnabled ) {
		
		DbgOut(( "[ImmVibeSPI] : ImmVibeSPI_ForceOut_AmpEnable\n"));
		
    	g_bAmpEnabled = true;

		// Generate PWM CLK
		vib_generatePWM(NvOdmPwmMode_Enable);

		// Enable GPIO(enable pin)
		vib_enable(NV_TRUE);
	}

	return VIBE_S_SUCCESS;
}

/* Called at initialization time to set PWM freq, disable amp, etc... */
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_Initialize( void )
{
	DbgOut(( "[ImmVibeSPI] : ImmVibeSPI_ForceOut_Initialize\n" ));

	vib_init();

	g_bAmpEnabled = true;   /* to force ImmVibeSPI_ForceOut_AmpDisable disabling the amp */

   	ImmVibeSPI_ForceOut_AmpDisable( 0 );

	return VIBE_S_SUCCESS;
}

/* Called at termination time to set PWM freq, disable amp, etc... */
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_Terminate( void )
{
 	DbgOut(( "[ImmVibeSPI] : ImmVibeSPI_ForceOut_Terminate\n" ));

	ImmVibeSPI_ForceOut_AmpDisable(0);

	return VIBE_S_SUCCESS;
}

/*
** Called by the real-time loop to set PWM duty cycle, and enable amp if required
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_Set( VibeUInt8 nActuatorIndex, VibeInt8 nForce )
{
    return VIBE_S_SUCCESS;
}

/*
** Called by the real-time loop to set force output, and enable amp if required
*/

IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_SetSamples( VibeUInt8 nActuatorIndex, VibeUInt16 nOutputSignalBitDepth, VibeUInt16 nBufferSizeInBytes, VibeInt8* pForceOutputBuffer )
{
    VibeInt8 nForce;

	NvU32	DutyCycle;
	/* NvU32	ReturnedPeriod; */

    switch (nOutputSignalBitDepth)
    {
        case 8:
            /* pForceOutputBuffer is expected to contain 1 byte */
            if (nBufferSizeInBytes != 1)
			{
//				DbgOut((KERN_ERR "[ImmVibeSPI] ImmVibeSPI_ForceOut_SetSamples nBufferSizeInBytes =  %d \n", nBufferSizeInBytes ));
				return VIBE_E_FAIL;
            }
            nForce = pForceOutputBuffer[0];
            break;
        case 16:
            /* pForceOutputBuffer is expected to contain 2 byte */
            if (nBufferSizeInBytes != 2) return VIBE_E_FAIL;

            /* Map 16-bit value to 8-bit */
            nForce = ((VibeInt16*)pForceOutputBuffer)[0] >> 8;
            break;
        default:
            /* Unexpected bit depth */
            return VIBE_E_FAIL;
    }
	 
//	DbgOut(( "[ImmVibeSPI] ImmVibeSPI_ForceOut_Set nForce =  %d ,CURRENT_TIME = %d\n", nForce , CURRENT_TIME));

#if 1
	//DbgOut(( "[ImmVibeSPI] : nForce = %d\n",nForce));
	if(haptic_state==ENABLE_HAPTIC) // levi debug
	{
		if ( nForce == 0 ) 
		{
			ImmVibeSPI_ForceOut_AmpDisable(1);
			DutyCycle = g_requestedPeriod >> 1; // 50% duty
		} 
		else 
		{
			ImmVibeSPI_ForceOut_AmpEnable(1);
			DutyCycle=((nForce + 128) * g_requestedPeriod) >> 8;
		
//		DbgOut(( "[ImmVibeSPI] ImmVibeSPI_ForceOut_Set DutyCycle =  %d \n", DutyCycle ));
		}
	
		pwm_config(haptic_vibrator->pwm, DutyCycle, g_requestedPeriod);
	}

#endif


	return VIBE_S_SUCCESS;

}

/*
** Called to set force output frequency parameters
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_SetFrequency( VibeUInt8 nActuatorIndex, VibeUInt16 nFrequencyParameterID, VibeUInt32 nFrequencyParameterValue )
{
   	/* This function is not called for ERM device */
	return VIBE_S_SUCCESS;
}

/*
** Called to get the device name (device name must be returned as ANSI char)
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_Device_GetName( VibeUInt8 nActuatorIndex, char *szDevName, int nSize )
{
   	return VIBE_S_SUCCESS;
}
