/*
 * (C) Copyright 2013
 * Device Solutions Ltd.
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <common.h>
#include <command.h>
#include <asm/byteorder.h>
#include <malloc.h>
#include <asm/arch/gpio.h>
#include <asm/arch/vybrid-regs.h>
#include <asm/arch/vybrid-pins.h>
#include <asm/arch/iomux.h>

/*
 * IO Test functions
 */

// IO pins
// LED1 - PTD29 - GPIO_65 - GP2_1
// LED2 - PTD30 - GPIO_64 - GP2_0
// BTN1 - PTD24 - GPIO_70 - GP2_6
// BTN2 - PTD25 - GPIO_69 - GP2_5
// OUT1 - PTB23 - GPIO_93 - GP2_29
// OUT2 - PTB22 - GPIO_44 - GP1_12
// IN1  - PTB20 - GPIO_42 - GP1_10
// IN2  - PTB21 - GPIO_43 - GP1_11

#define GPIO_OUTPUT_PAD_SETUP	0x000000C2
#define GPIO_INPUT_PAD_SETUP	0x0000003D

#define LED1 (65)
#define LED2 (64)
#define BTN1 (70)
#define BTN2 (69)
#define OUT1 (93)
#define OUT2 (44)
#define IN1  (42)
#define IN2  (43)

// GPIO code is not implemented elsewhere in u-boot for Vybrid
// Quick and dirty implementation here for now

// gpio_regs is defined

#define GPIO_BASE_ADDRESS  0x400FF000
#define GPIO_STRUCT_STRIDE 0x00000040
#define GPIO_PINS_PER_PORT 32
#define GPIO_PORT(gpio) ((gpio_regs*)(GPIO_BASE_ADDRESS+((gpio/GPIO_PINS_PER_PORT)*GPIO_STRUCT_STRIDE)))
#define GPIO_PIN_MASK(gpio) (1 << (gpio % GPIO_PINS_PER_PORT))

void ProcessGpio( int input, int output, int* pCurrentStatus, int* pStatusChanged )
{
	int new_status = 0;
	if( (GPIO_PORT(input)->gpio_pdir & GPIO_PIN_MASK(input)) > 0 )
		new_status = 1;
	if( *pCurrentStatus != new_status )
	{
		*pStatusChanged = 1;
		*pCurrentStatus = new_status;
	}

	if( new_status == 1 )
		GPIO_PORT(output)->gpio_psor = GPIO_PIN_MASK(output);
	else
		GPIO_PORT(output)->gpio_pcor = GPIO_PIN_MASK(output);
}

int io_test()
{
	int button1_status = 0;
	int button2_status = 0;
	int input1_status  = 0;
	int input2_status  = 0;
	int key_pressed    = 0;
	int status_changed = 0;

	printf( "IO Test in progress\n" );
	printf( "Press USER BUTTONs or activate INPUTS to test\n" );
	printf( "Press any key to end\n" );

	// Setup GPIO PADS
	__raw_writel(GPIO_INPUT_PAD_SETUP, IOMUXC_PAD_069);
	__raw_writel(GPIO_INPUT_PAD_SETUP, IOMUXC_PAD_070);
	__raw_writel(GPIO_INPUT_PAD_SETUP, IOMUXC_PAD_042);
	__raw_writel(GPIO_INPUT_PAD_SETUP, IOMUXC_PAD_043);
	__raw_writel(GPIO_OUTPUT_PAD_SETUP, IOMUXC_PAD_065);	
	__raw_writel(GPIO_OUTPUT_PAD_SETUP, IOMUXC_PAD_064);	
	__raw_writel(GPIO_OUTPUT_PAD_SETUP, IOMUXC_PAD_093);	
	__raw_writel(GPIO_OUTPUT_PAD_SETUP, IOMUXC_PAD_044);

	do
	{
		udelay(20000);  // 20ms?

		// Read inputs and reflect in outputs.  Report if there is a change
		ProcessGpio( IN1,  OUT1, &input1_status, &status_changed );
		ProcessGpio( IN2,  OUT2, &input2_status, &status_changed );
		ProcessGpio( BTN1, LED1, &button1_status, &status_changed );
		ProcessGpio( BTN2, LED2, &button2_status, &status_changed );

		if( status_changed == 1 )
		{
			printf( "IN1=%d INT2=%d BTN1=%d BTN2=%d\n", input1_status, input2_status, button1_status, button2_status );
			status_changed = 0;
		}

		if (tstc()) 	/* we got a key press	*/
		{
			(void) getc();  /* consume input	*/
			key_pressed = 1;
		}
	} while( key_pressed == 0 );

	return 1;
}


/*
 * Sub command handling
 */

static int do_qtest_io(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	return io_test();
}

static int do_qtest_capbuttons(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	return 0;
}

static int do_qtest_accel(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	return 0;
}

static int do_qtest_aout(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	return 0;
}

static int do_qtest_ain(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	return 0;
}



static cmd_tbl_t cmd_qtest_sub[] = {
	U_BOOT_CMD_MKENT(io, 3, 0, do_qtest_io, "", ""),
	U_BOOT_CMD_MKENT(capbuttons, 5, 0, do_qtest_capbuttons, "", ""),
	U_BOOT_CMD_MKENT(accel, 5, 0, do_qtest_accel, "", ""),
	U_BOOT_CMD_MKENT(aout, 5, 0, do_qtest_aout, "", ""),
	U_BOOT_CMD_MKENT(ain, 5, 0, do_qtest_ain, "", "")
};

/*
 * Subroutine:  do_qtest
 *
 * Description: Handler for 'qtest' command..
 *
 * Inputs:	argv[1] contains the subcommand
 *
 * Return:      None
 *
 */
static int do_qtest(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	cmd_tbl_t *c;

	/* Strip off leading 'qtest' command argument */
	argc--;
	argv++;

	c = find_cmd_tbl(argv[0], &cmd_qtest_sub[0], ARRAY_SIZE(cmd_qtest_sub));

	if (c)
		return  c->cmd(cmdtp, flag, argc, argv);
	else
		return CMD_RET_USAGE;
}

U_BOOT_CMD(
	qtest,	2,	2,	do_qtest,
	"Hardare tests for Quartz Dev Kit",
	""
);


