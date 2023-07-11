/* -- add for QCOM Platform */
#ifndef _es9218_COMMON_H_
#define _es9218_COMMON_H_

/* regsiters map */
#define ES9218_CONTROL_REG00	0x00 /* system settings */
#define ES9218_CONTROL_REG01	0x01 /* input selection and deemphasis */
#define ES9218_CONTROL_REG02	0x02
#define ES9218_CONTROL_REG03	0x03
#define ES9218_CONTROL_REG04	0x04 /* volume control1 by software */
#define ES9218_CONTROL_REG05	0x05 /* volume control2 by software */
#define ES9218_CONTROL_REG06	0x06 /* volume control3 by software */
#define ES9218_CONTROL_REG07	0x07 /* general settings */
#define ES9218_CONTROL_REG08	0x08 /* GPIO 1 Configuration */
#define ES9218_CONTROL_REG09	0x09
#define ES9218_CONTROL_REG10	0x0A
#define ES9218_CONTROL_REG11	0x0B /* channel mapping */
#define ES9218_CONTROL_REG12	0x0C /* DPLL/ASRC Settings */
#define ES9218_CONTROL_REG13	0x0D /* Moudlator Settings */
#define ES9218_CONTROL_REG14	0x0E /* Soft Start Settings */
#define ES9218_CONTROL_REG15	0x0F /* volume1 */
#define ES9218_CONTROL_REG16	0x10 /* volume2 */
#define ES9218_CONTROL_REG17	0x11
#define ES9218_CONTROL_REG18	0x12
#define ES9218_CONTROL_REG19	0x13
#define ES9218_CONTROL_REG20	0x14
#define ES9218_CONTROL_REG21	0x15
#define ES9218_CONTROL_REG22	0x16
#define ES9218_CONTROL_REG23	0x17
#define ES9218_CONTROL_REG24	0x18
#define ES9218_CONTROL_REG25	0x19
#define ES9218_CONTROL_REG26	0x1A
#define ES9218_CONTROL_REG27	0x1B
#define ES9218_CONTROL_REG28	0x1C
#define ES9218_CONTROL_REG29	0x1D
#define ES9218_CONTROL_REG30	0x1E
#define ES9218_CONTROL_REG31  0x1F
#define ES9218_CONTROL_REG32	0x20
#define ES9218_CONTROL_REG33	0x21
#define ES9218_CONTROL_REG34	0x22
#define ES9218_CONTROL_REG35	0x23
#define ES9218_CONTROL_REG36	0x24
#define ES9218_CONTROL_REG37	0x25
#define ES9218_CONTROL_REG38	0x26
#define ES9218_CONTROL_REG39	0x27
#define ES9218_CONTROL_REG40	0x28
#define ES9218_CONTROL_REG41  0x29
#define ES9218_CONTROL_REG42	0x2A
#define ES9218_CONTROL_REG43	0x2B
#define ES9218_CONTROL_REG44	0x2C
#define ES9218_CONTROL_REG45	0x2D
#define ES9218_CONTROL_REG46	0x2E
#define ES9218_CONTROL_REG47	0x2F
#define ES9218_CONTROL_REG48  0x30
#define ES9218_CONTROL_REG64	0x40 /* 4:2 chip id */
#define ES9218_CONTROL_REG65	0x41 /* gpio status */
#define ES9218_REG_MAX              255
#define meitu_AUDIO_DEBUG 1
#define ES9218_I2C_RETRIES  5
#define ES9218_CHIP_ID 0xC0
#define ES9038Q2M_CHIP_ID 0x70   /*hmct added, 9038 chip id */
#define ES9038K2M_CHIP_ID 0x30   /*hmct added, 9038 chip id */
#define ES9218_ID_REG 0x40
#define ES9218_I2C_REMAP  1

//MASK
#define ES9218_SERIAL_BITS_MASK    (~( 3<< 4))
#define ES9218_SERIAL_BITS_16   (0<<4)
#define ES9218_SERIAL_BITS_24   (1<<4)
#define ES9218_SERIAL_BITS_32   (2<<4)
#define ES9218_SERIAL_LENGTH_MASK  (~(3<<6))
#define ES9218_SERIAL_LENGTH_16   (0<<6)
#define ES9218_SERIAL_LENGTH_24   (1<<6)
#define ES9218_SERIAL_LENGTH_32   (2<<6)
struct es9218_regulator {
	const char *name;
};

struct es9218_dev_platform_data{
	char *driver_name;
	int rst_gpio;
	int pm_conv_gpio;
	struct es9218_regulator *vd_regulator, *va_regulator;
};

struct es9218_reg_peer {
	u8 addr;
	u8 val;
};

struct es9218_params {
	unsigned int size;
	struct es9218_reg_peer *peer;
	unsigned int mode;
};
extern int es9218_hph_switch_get(void);
#endif