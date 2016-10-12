#include <common.h>
#include <command.h>
#include <asm/arch/hardware.h>
#include <config_cmd_default.h>
#include <asm/io.h>

/* Added by MYIR, for boot device checking. */
#define CONTROL_MODULE_BASE	0x44E10000
#define CONTROL_STATUS		(CONTROL_MODULE_BASE + 0x40)
#define SYSBOOT_MASK		0x1F
#define SDBOOT_FLAG		0x16
#define SDBOOT_FLAG1     0x1C

#define PRINT_LINE		printf("FUNC:%s(), LINE:%d\n", __func__, __LINE__)

int do_chkbootdev(cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])  
{
	/* MYIR */
	unsigned int reg_sysboot = 0;

	reg_sysboot = __raw_readl(CONTROL_STATUS) & SYSBOOT_MASK;

	printf ("sysboot[4-0]: 0x%02X ... ", reg_sysboot);

	if (reg_sysboot == SDBOOT_FLAG
		|| reg_sysboot == SDBOOT_FLAG1) {
		printf("sdboot.\n");
		return 0;
	} else {
		printf("nandboot.\n");
		return 1;
	}
}

U_BOOT_CMD(
        chkbootdev ,      1,      0,      do_chkbootdev,
        " check boot device",
        " - check boot device.\n"
	"return value:\n"
	"  TRUE(0)  - sd boot\n"
	"  FALSE(1) - nand boot\n"
);

