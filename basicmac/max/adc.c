

#include "hw.h"
#include "adc_regs.h"
#include "clkman.h"
#include "pwrman_regs.h"
#include "mxc_sys.h"


// TODO: implement
/*
static void adc_on (void) {

}

static void adc_off (void) {

}
*/

unsigned int adc_read (unsigned int chnl, unsigned int rate) {
	  int err;
	  unsigned int adc_scale = 1;
	  unsigned int result = 0;

	  chnl = 0; //Todo: make sure channel is OK

	  err = SYS_ADC_Init();
	  ASSERT(err==0);

	  /* Wipe previous configuration */
	  MXC_ADC->intr = 0;

	  /* Clear all ADC interrupt flags (W1C) */
	  MXC_ADC->intr = MXC_ADC->intr;

	  /* Enable done interrupt */
	  MXC_ADC->intr = MXC_F_ADC_INTR_ADC_DONE_IE;

	  /* Power up the ADC */
	  MXC_ADC->ctrl = (MXC_F_ADC_CTRL_ADC_PU |
			   MXC_F_ADC_CTRL_ADC_CLK_EN |
			   MXC_F_ADC_CTRL_BUF_PU |
			   MXC_F_ADC_CTRL_ADC_REFBUF_PU |
			   MXC_F_ADC_CTRL_ADC_CHGPUMP_PU);

	  // void ADC_StartConvert(mxc_adc_chsel_t channel, unsigned int adc_scale, unsigned int bypass)
	  uint32_t ctrl_tmp;

	  /* Clear the ADC done flag */
	  //ADC_ClearFlags(MXC_F_ADC_INTR_ADC_DONE_IF);

	  /* Insert channel selection */
	  ctrl_tmp = MXC_ADC->ctrl;
	  ctrl_tmp &= ~(MXC_F_ADC_CTRL_ADC_CHSEL);
	  ctrl_tmp |= ((chnl << MXC_F_ADC_CTRL_ADC_CHSEL_POS) & MXC_F_ADC_CTRL_ADC_CHSEL);

	  /* Clear channel configuration */
	  ctrl_tmp &= ~(MXC_F_ADC_CTRL_ADC_REFSCL | MXC_F_ADC_CTRL_ADC_SCALE | MXC_F_ADC_CTRL_BUF_BYPASS);

	  /* ADC reference scaling must be set for all channels but two*/
	  if ((chnl != MXC_V_ADC_CTRL_ADC_CHSEL_VDD18) && (chnl != MXC_V_ADC_CTRL_ADC_CHSEL_VDD12)) {
	    ctrl_tmp |= MXC_F_ADC_CTRL_ADC_REFSCL;
	  }

	  /* Finalize user-requested channel configuration */
	  if (adc_scale || chnl > MXC_V_ADC_CTRL_ADC_CHSEL_AIN3) {
	    ctrl_tmp |= MXC_F_ADC_CTRL_ADC_SCALE;
	  }


	  /* Write this configuration */
	  MXC_ADC->ctrl = ctrl_tmp;

	  /* Start conversion */
	  MXC_ADC->ctrl |= MXC_F_ADC_CTRL_CPU_ADC_START;



	  //int ADC_GetData(uint16_t *outdata)
		/* See if a conversion is in process */
		if (MXC_ADC->status & MXC_F_ADC_STATUS_ADC_ACTIVE) {
			/* Wait for conversion to complete */
			while ((MXC_ADC->intr & MXC_F_ADC_INTR_ADC_DONE_IF) == 0);
		}

		/* Read 32-bit value and truncate to 16-bit for output depending on data align bit*/
		if((MXC_ADC->ctrl & MXC_F_ADC_CTRL_ADC_DATAALIGN) == 0)
				result = (uint16_t)(MXC_ADC->data); /* LSB justified */
		else
				result = (uint16_t)(MXC_ADC->data >> 6); /* MSB justified */

	//	/* Check for overflow */
	//	if (MXC_ADC->status & MXC_F_ADC_STATUS_ADC_OVERFLOW) {
	//		return E_OVERFLOW;
	//	}


	  //Shutdown ADC

	  /* Power down the ADC */
	  MXC_ADC->ctrl &= ~(MXC_F_ADC_CTRL_ADC_PU |
			   MXC_F_ADC_CTRL_ADC_CLK_EN |
			   MXC_F_ADC_CTRL_BUF_PU |
			   MXC_F_ADC_CTRL_ADC_REFBUF_PU |
			   MXC_F_ADC_CTRL_ADC_CHGPUMP_PU);

	  /* Power down the ADC AFE, enable clocks */
	  MXC_PWRMAN->pwr_rst_ctrl &= ~(MXC_F_PWRMAN_PWR_RST_CTRL_AFE_POWERED);
	  MXC_CLKMAN->clk_ctrl &= ~(MXC_F_CLKMAN_CLK_CTRL_ADC_CLOCK_ENABLE);

		//Return
	return result;
}
