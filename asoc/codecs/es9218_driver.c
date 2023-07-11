#include <linux/module.h>
#include <linux/init.h>
#include <linux/firmware.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/printk.h>
#include <linux/ratelimit.h>
#include <linux/debugfs.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/tlv.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/pm_runtime.h>
#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/wait.h>
#include <linux/regulator/consumer.h>
#include <linux/i2c.h>
#include <linux/of_gpio.h>
#include <linux/of.h>
#include <linux/clk.h>
#include "es9218_driver.h"
#include <linux/of_gpio.h>

#ifdef CONFIG_HISENSE_PRODUCT_DEVINFO
#include <linux/productinfo.h>
#endif	/* CONFIG_HISENSE_PRODUCT_DEVINFO */

#undef pr_debug
#define pr_debug pr_err
#undef pr_info
#define pr_info pr_err

#define ES9218_DEV_NAME "HiFi-es9218"
#define ES9218_RATES SNDRV_PCM_RATE_8000_384000
#define ES9218_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | \
                                    SNDRV_PCM_FMTBIT_S24_LE | \
                                    SNDRV_PCM_FMTBIT_S32_LE)

#ifdef CONFIG_HISENSE_PRODUCT_DEVINFO
#define HIFI_INFO_LEN    50
#endif /* CONFIG_HISENSE_PRODUCT_DEVINFO */

static unsigned char dac_parameters[][2] = {
	{1, 0x80},
	{2, 0x34},
	{3, 0x70},
	{4, 0x0},
	{5, 0x68},
	{6, 0x07},
	{7, 0x00},
	{8, 0xdd},
	{9, 0x0},
	{10, 0x02},
	{11, 0x0},
	{12, 0xff},
	{13, 0x0},
	{14, 0x25},//CHANGED
	{15, 0x0},
	{16, 0x0},
	{17, 0xff},
	{18, 0xff},
	{19, 0xff},
	{20, 0x62},
	{21, 0x0},
	{22, 0xc0},
	{23, 0x05},
	{24, 0x0},
	{25, 0x0},
	{26, 0x62},
	{27, 0xc4},
	{28, 0xf0},
	{29, 0x50},//CHANGED
	{30, 0x0},
	{31, 0x0},
	{32, 0x10},//CHANGED
	{33, 0x3c},
	{34, 0x0},
	{35, 0x0},
	{36, 0x0},
	{37, 0x0},
	{38, 0x0},
	{39, 0x0},
	{40, 0x0},
	{41, 0x0},
	{42, 0x0},
	{43, 0x0},
	{44, 0x0},
	{45, 0x0},
	{46, 0x0},
	{47, 0x0},
	{48, 0x8b},//CHANGED
};

static unsigned char dsd_volume[][2] = {
	{0, 0xff},
	{1, 0x74},
	{2, 0x6f},
	{3, 0x6a},
	{4, 0x65},
	{5, 0x5f},
	{6, 0x5c},
	{7, 0x56},
	{8, 0x51},
	{9, 0x4c},
	{10, 0x47},
	{11, 0x43},
	{12, 0x3f},
	{13, 0x3b},
	{14, 0x37},
	{15, 0x34},
	{16, 0x2f},
	{17, 0x2b},
	{18, 0x27},
	{19, 0x23},
	{20, 0x20},
	{21, 0x1d},
	{22, 0x1a},
	{23, 0x17},
	{24, 0x14},
	{25, 0x11},
	{26, 0x0e},
	{27, 0x0b},
	{28, 0x09},
	{29, 0x06},
	{30, 0x03},
	{31, 0x00},
};

/*hmct modified, for es9038q2m*/
static unsigned char es9038q2m_init_register[][2] = {
	{0x00, 0x00},
	{0x01, 0x8c},  /* I2S input */ //{0x01, 0x80},  /* I2S input */
	{0x02, 0x34},
	{0x03, 0x40},
	{0x04, 0x00},
	{0x05, 0x68},
	{0x06, 0x42},  
	{0x07, 0x80},   /* DIGI_FILT_SEL */
	{0x08, 0xdd},
	{0x09, 0x22},  
	{0x0a, 0x02},
	{0x0b, 0x00},
	{0x0c, 0x5a},
	{0x0d, 0x00},
	{0x0e, 0x8a},
	{0x0f, 0x00},  /*VOL_CTRL_L */
	
	{0x10, 0x00},  /*VOL_CTRL_R */
	{0x11, 0xff},
	{0x12, 0xff},
	{0x13, 0xff},
	{0x14, 0x7f},
	{0x15, 0x00},
	{0x16, 0xf5},  /*2nd_HARMONIC_COMPENSATION_L */
	{0x17, 0xff},  /*2nd_HARMONIC_COMPENSATION_H */
	{0x18, 0x00},  /*3rd_HARMONIC_COMPENSATION_L */
	{0x19, 0x00},  /*3rd_HARMONIC_COMPENSATION_H */
	{0x1a, 0x62},
	{0x1b, 0xd4},
	{0x1c, 0xf0},
	{0x1d, 0x00},
	{0x1e, 0x00},
	{0x1f, 0x00},
	
	{0x20, 0x00},
	{0x21, 0x3c},
	{0x22, 0x00},
	{0x23, 0x00},
	{0x24, 0x00},
	{0x25, 0x00},
	{0x26, 0x00},
	{0x27, 0x40},
	{0x28, 0x00},
	{0x29, 0x00},
	{0x2a, 0x00},
	{0x2b, 0x00},
	{0x2c, 0x00},
	{0x2d, 0x05},
	{0x2e, 0x00},
};

/*hmct modified, for es9038k2m*/
static unsigned char es9038k2m_init_register[][2] = {
	{0x00, 0x00},
	{0x01, 0x8c},  /* I2S input */ //{0x01, 0x80},  /* I2S input */
	{0x02, 0x18},
	{0x03, 0x10},
	{0x04, 0x00},
	{0x05, 0x68},
	{0x06, 0x4A},  
	{0x07, 0x80},   /* DIGI_FILT_SEL */
	{0x08, 0x10},
	{0x09, 0x22},  
	{0x0a, 0x05},
	{0x0b, 0x02},
	{0x0c, 0x5a},
	{0x0d, 0x00},
	{0x0e, 0x8a},
	{0x0f, 0x00},  /*VOL_CTRL_L */
	
	{0x10, 0x00},  /*VOL_CTRL_R */
	{0x11, 0xff},
	{0x12, 0xff},
	{0x13, 0xff},
	{0x14, 0x7f},
	{0x15, 0x00},
	{0x16, 0x00},  /*2nd_HARMONIC_COMPENSATION_L */
	{0x17, 0x00},  /*2nd_HARMONIC_COMPENSATION_H */
	{0x18, 0x00},  /*3rd_HARMONIC_COMPENSATION_L */
	{0x19, 0xF5},  /*3rd_HARMONIC_COMPENSATION_H */	
};


#ifdef CONFIG_DEBUG_FS
static struct dentry *es9218_debugfs_root;
static struct es9218_data *current_debugfs;
#endif
struct audio_params {
    unsigned int i2s_format;
    unsigned int pcm_format;
    unsigned int rate;
    struct es9218_params  *private_params;
};
struct es9218_data {
	struct i2c_client *client;
#ifdef ES9218_I2C_REMAP
	struct regmap *regmap;
#endif
	struct device *dev;

	char *driver_name;
	struct snd_soc_codec *codec;
	/* state */
	bool on;
	/*
	* work_mode here is a unsigned int type,
	* bit 0 ~ 7 for headphone impedance,
	* bit 8 ~ 9 for channel mode
	* bit 12 for I2S Master Mode
	* other bits reserved.
	*/
	struct mutex lock;
	struct audio_params params;   //?
	int param_row;
	int param_col;
	unsigned int *ic_params;

#ifdef CONFIG_DEBUG_FS
	struct dentry *debugfs_reg;
#endif
	struct regulator *hifi_dac_regulator;
	int rst_gpio;
	int hifi_sel_gpio;
	int hph_sel_gpio;
	/*ZSQ add oscillator gpio control */
	int osci_gpio;	
	int dac_pwr_gpio;	
	int pa_pwr_gpio;	
	int hifi_status;
	struct clk *hifi_mclk;
	int sysclk;
};

static  struct es9218_data *pes9218_data;
static u8 Chipid=0;
 enum{
    ES9218_STATUS_STANDBY,
    ES9218_STATUS_BYPASS,
    ES9218_STATUS_HIFI 
 };
 static int es9218_hifi_control = ES9218_STATUS_HIFI;
 enum{
    ES9038_Q2M,
	ES9038_K2M,
 };
 static int es9038_type = ES9038_Q2M;
#ifndef ES9218_I2C_REMAP
static int es9218_i2c_read_byte(struct es9218_data *edata, u8 reg, u8* reg_data)
{
	int ret = 0;
	u8 buf = 0xff;

	ret = i2c_master_send(edata->client, &reg, 1);
	if (ret < 0) {
		dev_err(&edata->client->dev, "%s i2c send cmd error reg: %d.\n",
			__func__, reg);
		return ret;
	}

	ret = i2c_master_recv(edata->client, reg_data, 1);
	if (ret < 0) {
		dev_err(&edata->client->dev, "%s i2c recv error.\n ",
			__func__);
		return ret;
	}

	return ret;
}

static u8 es9218_i2c_write_byte(struct es9218_data *edata,u8 reg, u8 data)
{
	int ret = 0;
	u8 cmd[2];

	cmd[0] = reg;
	cmd[1] = data;

	if (!edata->client || !edata->client->addr) {
		pr_err("%s: client is NULL.\n ", __func__);
		return -EFAULT;
	}

	//edata->client->addr = addr;   ?

	ret = i2c_master_send(edata->client, cmd, sizeof(cmd));
	if (ret < 1) {
		dev_err(&edata->client->dev, "%s i2c send error,"
			"cmd[0]: %d, cmd[1]: %d\n", __func__,
			cmd[0], cmd[1]);
	}

	dev_info(&edata->client->dev, "%s: i2c send addr: 0x%x,"
		"cmd[0]: %d, cmd[1]: %d\n", __func__,
		edata->client->addr, cmd[0], cmd[1]);

	return ret;
}
#endif
static int es9218_write_byte(struct es9218_data *es9218, unsigned char reg_addr, unsigned char reg_data)
{
		int ret = -1;
		unsigned char cnt = 0;
	
		while(cnt < ES9218_I2C_RETRIES) {
#ifdef ES9218_I2C_REMAP
		ret = regmap_write(es9218->regmap, reg_addr, reg_data);
		if(ret < 0) {
			pr_err("%s: regmap_write cnt=%d error=%d\n", __func__, cnt, ret);
			pr_err("%s: regmap addr=%d,data=%d\n",__func__,reg_addr,reg_data);
		} else {
			break;
		}
#else
		ret = es9218_i2c_write_byte(es9218, reg_addr, reg_data);
		if(ret < 0) {
			pr_err("%s: i2c_write cnt=%d error=%d\n", __func__, cnt, ret);
		} else {
			break;
		}
#endif
			cnt ++;
		}
	
		return ret;
}
static int es9218_read_byte(struct es9218_data *es9218,unsigned int reg_addr)
{
    int ret = -1;
    unsigned char cnt = 0;
    unsigned int reg_data;
    while(cnt < ES9218_I2C_RETRIES) {
#ifdef ES9218_I2C_REMAP
    ret = regmap_read(es9218->regmap, reg_addr, &reg_data);
    if(ret < 0) {
        pr_err("%s: regmap_read cnt=%d error=%d\n", __func__, cnt, ret);
    } else {
        break;
    }
#else
    ret= es9218_i2c_read_byte(es9218, reg_addr, &reg_data);
    if(ret < 0) {
        pr_err("%s: i2c_read cnt=%d error=%d\n", __func__, cnt, ret);
    } else {
        break;
    }
#endif
        cnt ++;
    }

    return reg_data;
}


/******************************************************
 *
 * Digital Audio Interface
 *
 ******************************************************/
#if 0
 static void es9218_HiFi_chip_on(struct es9218_data *edata)
{
    pr_info("%s get enter\n",__func__); 
	es9218_write_byte(edata,ES9218_CONTROL_REG14, 0x45);
	es9218_write_byte(edata,ES9218_CONTROL_REG02, 0xB4);	 
	es9218_write_byte(edata,ES9218_CONTROL_REG05, 0x0);
	es9218_write_byte(edata,ES9218_CONTROL_REG04, 0xFF);
	es9218_write_byte(edata,ES9218_CONTROL_REG32, 0x80);
	usleep_range(3000, 3200);
	es9218_write_byte(edata,ES9218_CONTROL_REG29, 0x0D);
	usleep_range(5000, 5200);
	es9218_write_byte(edata,ES9218_CONTROL_REG46, 0x80);
	usleep_range(3000, 3200);
	es9218_write_byte(edata,ES9218_CONTROL_REG32, 0x83);
	usleep_range(120000, 121000);
	es9218_write_byte(edata,ES9218_CONTROL_REG46, 0x0);
	es9218_write_byte(edata,ES9218_CONTROL_REG05, 0x7F);
	usleep_range(5000, 5200);
	es9218_write_byte(edata,ES9218_CONTROL_REG04, 0x0);
	es9218_write_byte(edata,ES9218_CONTROL_REG15, 0x0);
	es9218_write_byte(edata,ES9218_CONTROL_REG16, 0x0);
	es9218_write_byte(edata,ES9218_CONTROL_REG27, 0xC4);
}

static void es9218_HiFi_chip_off(struct es9218_data *edata)
{
    pr_info("%s get enter\n",__func__); 
	es9218_write_byte(edata,ES9218_CONTROL_REG14, 0x5);
	es9218_write_byte(edata,ES9218_CONTROL_REG02, 0xB4);
	es9218_write_byte(edata,ES9218_CONTROL_REG05, 0x0);
	es9218_write_byte(edata,ES9218_CONTROL_REG04, 0xFF);
	es9218_write_byte(edata,ES9218_CONTROL_REG20, 0xFF);
	usleep_range(50000, 52000);
	es9218_write_byte(edata,ES9218_CONTROL_REG32, 0x80);
	usleep_range(5000, 5200);
	es9218_write_byte(edata,ES9218_CONTROL_REG29, 0x0D);
	usleep_range(10000,15000); 	
}
#endif 

static int es9218_startup(struct snd_pcm_substream *substream,
        struct snd_soc_dai *dai)
{
   struct es9218_data *es9218 = snd_soc_component_get_drvdata(dai->component);

    pr_info("%s: enter\n  %s", __func__,es9218->driver_name);
    //aw8898_run_pwd(aw8898, false);

    return 0;
}

static int es9218_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
    struct es9218_data *es9218 = snd_soc_component_get_drvdata(dai->component);

    pr_info("%s: fmt=0x%x\n", __func__, fmt);

    /* Supported mode: regular I2S, slave, or PDM */
    switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
    case SND_SOC_DAIFMT_I2S:
        if ((fmt & SND_SOC_DAIFMT_MASTER_MASK) != SND_SOC_DAIFMT_CBS_CFS) {
            dev_err(es9218->dev, "%s: invalid codec master mode\n", __func__);
            return -EINVAL;
        }
        break;
    default:
        dev_err(es9218->dev, "%s: unsupported DAI format %d\n", __func__,
                fmt & SND_SOC_DAIFMT_FORMAT_MASK);
        return -EINVAL;
    }
    return 0;
}
static int es9218_set_dai_sysclk(struct snd_soc_dai *dai,
        int clk_id, unsigned int freq, int dir)
{
    struct es9218_data *es9218 = snd_soc_component_get_drvdata(dai->component);

    pr_info("%s: freq=%d, %s\n", __func__, freq,es9218->driver_name);

    es9218->sysclk = freq;
    return 0;
}
static int es9218_hw_params(struct snd_pcm_substream *substream,
    struct snd_pcm_hw_params *params,
    struct snd_soc_dai *dai)
{
    unsigned int rate = 0;
    unsigned int serial_bits = 0;
    unsigned int serial_length=0;
    unsigned int reg_data=0;
    int width = 0;
   
    struct es9218_data  *es9218 = snd_soc_component_get_drvdata(dai->component);
    pr_debug("%s, enter  %s\n", __func__, es9218->driver_name);

    /* Supported */

    //get rate param
     rate = params_rate(params);
    pr_debug("%s: requested rate: %d, sample size: %d\n", __func__, rate,
       snd_pcm_format_width(params_format(params)));
    width=params_width(params);
    pr_debug("%s: requested width = %d \n",__func__,width);
    switch(width)
    {
        case 16:
            serial_bits = ES9218_SERIAL_BITS_16;
	     serial_length=ES9218_SERIAL_LENGTH_16;
            break;
        case 24:
            serial_bits = ES9218_SERIAL_BITS_24;
	     serial_length=ES9218_SERIAL_LENGTH_24;
            break;
        case 32:
            serial_bits = ES9218_SERIAL_BITS_32;
	     serial_length=ES9218_SERIAL_LENGTH_32;
            break;
        default:
            serial_bits = ES9218_SERIAL_BITS_16;
	     serial_length=ES9218_SERIAL_LENGTH_16;
            pr_err("%s: width can not support\n", __func__);
            break;
    }

    reg_data=es9218_read_byte(es9218,ES9218_CONTROL_REG01);
    pr_info("%s read 01-reg data=0x%x\n",__func__,reg_data);
#if 0

    reg_data &= ES9218_SERIAL_LENGTH_MASK;
    reg_data |= serial_length;
    pr_info("%s write 01-reg data=0x%x\n",__func__,reg_data);
    es9218_write_byte(es9218,ES9218_CONTROL_REG01,reg_data);
    reg_data=0;
//#else
    reg_data=es9218_read_byte(es9218,ES9218_CONTROL_REG02);
    pr_info("%s read 02-reg data=0x%x\n",__func__,reg_data);
    reg_data &= ES9218_SERIAL_BITS_MASK;
    reg_data |= serial_bits;
    pr_err("%s write 02-reg data=0x%x\n",__func__,reg_data);
    es9218_write_byte(es9218,ES9218_CONTROL_REG02,reg_data);
#endif
    return 0;
}

static int es9218_codec_mute(struct snd_soc_dai *dai, int mute, int stream)
{
   
    struct es9218_data *es9218 = snd_soc_component_get_drvdata(dai->component);
  
    pr_info("%s: mute state=%d,%s\n", __func__, mute,es9218->driver_name);
	
    if (mute) {
        gpio_set_value_cansleep(es9218->hifi_sel_gpio,1);
    } else {
        gpio_set_value_cansleep(es9218->hifi_sel_gpio,0);
    }
    return 0;
}

static void es9218_shutdown(struct snd_pcm_substream *substream,
        struct snd_soc_dai *dai)
{   
    struct  es9218_data *es9218= snd_soc_component_get_drvdata(dai->component);
    pr_debug("%s, enter  %s\n", __func__, es9218->driver_name);     
}


static const struct snd_soc_dai_ops es9218_dai_ops = {
    .startup = es9218_startup,
    .set_fmt = es9218_set_fmt,
    .set_sysclk = es9218_set_dai_sysclk,
    .hw_params = es9218_hw_params,
    .mute_stream = es9218_codec_mute,
    .shutdown = es9218_shutdown,
};
static struct snd_soc_dai_driver es9218_dai[] = {
    {
        .name = "es9218-aif",
        .id = 1,
        .playback = {
            .stream_name = "HiFi_Playback",
            .channels_min = 1,
            .channels_max = 2,
            .rates = ES9218_RATES,
            .formats = ES9218_FORMATS,
        },

        .capture = {
            .stream_name = "HiFi_Capture",
            .channels_min = 1,
            .channels_max = 2,
            .rates = ES9218_RATES,
            .formats = ES9218_FORMATS,
         },

        .ops = &es9218_dai_ops,
        .symmetric_rates = 1,
        .symmetric_channels = 1,
        .symmetric_samplebits = 1,
    },
};
/*****************************************************
 *
 * regmap
 *
 *****************************************************/
bool es9218_writeable_register(struct device *dev, unsigned int reg)
{
    /* enable read access for all registers */
    return 1;
}

bool es9218_readable_register(struct device *dev, unsigned int reg)
{
    /* enable read access for all registers */
    return 1;
}

bool es9218_volatile_register(struct device *dev, unsigned int reg)
{
    /* enable read access for all registers */
    return 1;
}

static const struct regmap_config es9218_regmap = {
    .reg_bits = 8,
    .val_bits = 8,
    .max_register = ES9218_REG_MAX,
    .writeable_reg = es9218_writeable_register,
    .readable_reg = es9218_readable_register,
    .volatile_reg = es9218_volatile_register,
    .cache_type = REGCACHE_RBTREE,
};
/*****************************************************
 *
 * codec weiget
 *
 *****************************************************/
 #if 1
 static void es9218_chip_init(struct es9218_data *edata);
 static const DECLARE_TLV_DB_SCALE(digital_gain,0,50,0);

   struct soc_mixer_control es9218_mixer ={
    .reg    = ES9218_CONTROL_REG15,
    .shift  = 0,
    .max    = 0,
    .min    = -255,
 };
 static int es9218_volume_info(struct snd_kcontrol *kcontrol,struct snd_ctl_elem_info *uinfo)
{
    struct soc_mixer_control *mc = (struct soc_mixer_control*) kcontrol->private_value;

    //set kcontrol info
    uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
    uinfo->count = 2;
    uinfo->value.integer.min = 0;
    uinfo->value.integer.max = mc->max - mc->min;
    return 0;
}
  static int es9218_volume_get(struct snd_kcontrol *kcontrol,struct snd_ctl_elem_value *ucontrol)
  {
  #if 1
	  struct snd_soc_component *component =snd_soc_kcontrol_component(kcontrol);
	  struct es9218_data *es9218 = snd_soc_component_get_drvdata(component);
	  unsigned int value = 0;


	  struct soc_mixer_control *mc = (struct soc_mixer_control*) kcontrol->private_value;
	  if( es9218_hifi_control == ES9218_STATUS_HIFI)  
	     value=es9218_read_byte(es9218, ES9218_CONTROL_REG15);
	  pr_info("%s:volume R value get %d\n",__func__,value);
	  ucontrol->value.integer.value[0] = (value >> mc->shift);
      if( es9218_hifi_control == ES9218_STATUS_HIFI)  
	     value=es9218_read_byte(es9218, ES9218_CONTROL_REG16);
	  pr_info("%s:volume L value get %d\n",__func__,value);
	  ucontrol->value.integer.value[1] = (value >> mc->shift);

#endif
	  return 0;
  }
  static int es9218_volume_put(struct snd_kcontrol *kcontrol,struct snd_ctl_elem_value *ucontrol)
{
#if 1

    struct soc_mixer_control *mc = (struct soc_mixer_control*) kcontrol->private_value;
    struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
    struct es9218_data *es9218 = snd_soc_component_get_drvdata(component);
    unsigned int value_l = 0;
    unsigned int value_r = 0;

    //value is right
    value_l= ucontrol->value.integer.value[0];
    value_r =ucontrol->value.integer.value[1];
    if(value_l> (mc->max-mc->min)|| value_l<0){
      pr_err("%s:value over range \n",__func__);
      return -1;
    }
    if(value_r> (mc->max-mc->min)|| value_r<0){
      pr_err("%s:value over range \n",__func__);
      return -1;
    }

    //smartpa have clk
    //cal real value
    es9218_write_byte(es9218, ES9218_CONTROL_REG15, value_l);
    pr_info("%s:volume L value put %d\n",__func__,value_l);
    //write value
    es9218_write_byte(es9218, ES9218_CONTROL_REG16, value_r);
    pr_info("%s:volume R value put %d\n",__func__,value_r);
#endif
    return 0;
}

static struct snd_kcontrol_new es9218_volume = {
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.name  = "es9218_rx_volume",
	.access= SNDRV_CTL_ELEM_ACCESS_TLV_READ|SNDRV_CTL_ELEM_ACCESS_READWRITE,
	.tlv.p	= (digital_gain),
	.info = es9218_volume_info,
	.get =	es9218_volume_get,
	.put =	es9218_volume_put,
	.private_value = (unsigned long)&es9218_mixer,
};
static const char *const hifi_function[] = { "Standby","Bypass","HiFi"};
static const char *const hifi_bit[] = { "bit_16","bit_24","bit_32"};
static const char *const hifi_master[] = { "Slave","Master"};
static const char *const hifi_clk_div[] = { "div_4","div_8","div_16","div_32"};

#ifdef CONFIG_HISENSE_PRODUCT_DEVINFO
static void hifi_register_productinfo(void)
{
	char hifi_info[HIFI_INFO_LEN];

	memset(hifi_info,0,sizeof(hifi_info));
	
	if ( es9038_type == ES9038_Q2M){
	   snprintf(hifi_info,HIFI_INFO_LEN,"ESS ESS9038 Q2M");
	}else{
	   snprintf(hifi_info,HIFI_INFO_LEN,"ESS ESS9038 K2M");
	}
	productinfo_register(PRODUCTINFO_CODEC_ID, hifi_info, NULL);
}
#endif /* CONFIG_HISENSE_PRODUCT_DEVINFO */



static void es9218_set_mode_HiFi(struct es9218_data *edata)
{
      pr_info("%s get enter\n",__func__);
      gpio_set_value_cansleep(edata->hifi_sel_gpio,1); 
	  gpio_set_value_cansleep(edata->hph_sel_gpio,1); 

      /*ZSQ add oscillator gpio control */
      gpio_set_value_cansleep(edata->osci_gpio,1); 
	  gpio_set_value_cansleep(edata->dac_pwr_gpio,1); 
	  gpio_set_value_cansleep(edata->pa_pwr_gpio,1); 
      usleep_range(10000,15000);
      gpio_set_value_cansleep(edata->rst_gpio,0);
      usleep_range(10000,15000);
      gpio_set_value_cansleep(edata->rst_gpio,1); 
      usleep_range(5000, 5200);
      gpio_set_value_cansleep(edata->rst_gpio,0);
      usleep_range(10000,15000);
      gpio_set_value_cansleep(edata->rst_gpio,1);	 

      es9218_chip_init(edata);
}
static void es9218_set_mode_Bypass(struct es9218_data *edata)
{
    int gpio_value;
	 pr_info("%s get enter\n",__func__);	

	 gpio_value = gpio_get_value_cansleep(edata->hifi_sel_gpio);
     if (!gpio_value) 
     {
	     gpio_set_value_cansleep(edata->hifi_sel_gpio,1);  
	 }
	 
	  usleep_range(100000,150000);
	  gpio_set_value_cansleep(edata->hph_sel_gpio,0); 
	  usleep_range(10000,15000);
	  gpio_value = gpio_get_value_cansleep(edata->rst_gpio);
	  if (gpio_value){
	  	  pr_info("%s hifi power off\n",__func__);
          gpio_set_value_cansleep(edata->rst_gpio,0);
	     /*ZSQ add oscillator gpio control */
         gpio_set_value_cansleep(edata->osci_gpio,0); 
	     gpio_set_value_cansleep(edata->dac_pwr_gpio,0); 
	     gpio_set_value_cansleep(edata->pa_pwr_gpio,0); 	    
	  }else{
           pr_info("%s rst_value = %d\n",__func__,gpio_value);
	  }         
     
}
/*hmct added, for disable headphone out*/
static void es9218_set_mode_standby(struct es9218_data *edata)
{
     int rst_value;
      pr_info("%s get enter\n",__func__); 

 /*ZSQ add using bypass setting to avoid earphone detect problem*/		  
      gpio_set_value_cansleep(edata->hifi_sel_gpio,1); 
	  gpio_set_value_cansleep(edata->hph_sel_gpio,1); 
	  
      rst_value = gpio_get_value_cansleep(edata->rst_gpio);
	  if (rst_value){
	  	  pr_info("%s hifi power off\n",__func__);
          gpio_set_value_cansleep(edata->rst_gpio,0);
	  /*ZSQ add oscillator gpio control */
          gpio_set_value_cansleep(edata->osci_gpio,0);
	      gpio_set_value_cansleep(edata->dac_pwr_gpio,0);
	      gpio_set_value_cansleep(edata->pa_pwr_gpio,0);
	  }else{
           pr_info("%s rst_value = %d\n",__func__,rst_value);
	  }      
}

static void es9218_mode_set(struct es9218_data *edata)
{
    pr_debug("%s hifi mode set=%d\n", __func__, edata->hifi_status);
    switch(edata->hifi_status)
    {
        case ES9218_STATUS_STANDBY:
	  es9218_set_mode_standby(edata);
	  pr_info("%s set hifi mode to standby\n",__func__);
	  break;
	 case ES9218_STATUS_BYPASS:
	 es9218_set_mode_Bypass(edata);
	 pr_info("%s set hifi mode to bypass\n",__func__);
	 break;
	 case ES9218_STATUS_HIFI:
	 es9218_set_mode_HiFi(edata);
	 pr_info("%s set hifi mode to hifi\n",__func__);
	 break;
	 default:
	 break;
    }
}

int es9218_hph_switch_get(void)
{
    int value;

	value = gpio_get_value_cansleep(pes9218_data->hph_sel_gpio);
    pr_debug("%s: gpio stauts=%d\n", __func__, value);
    
    return value;
}

EXPORT_SYMBOL(es9218_hph_switch_get);

static int es9218_hifi_get(struct snd_kcontrol *kcontrol,
        struct snd_ctl_elem_value *ucontrol)
{
    pr_debug("%s: es9218_hifi_status = %d\n", __func__, es9218_hifi_control);
    ucontrol->value.integer.value[0] = es9218_hifi_control;
    return 0;
}

static int es9218_hifi_set(struct snd_kcontrol *kcontrol,
        struct snd_ctl_elem_value *ucontrol)
{
    struct snd_soc_component *component =snd_soc_kcontrol_component(kcontrol);
    struct es9218_data *es9218 = snd_soc_component_get_drvdata(component);

    pr_debug("%s: ucontrol->value.integer.value[0]=%ld\n ",
            __func__, ucontrol->value.integer.value[0]);
    if(ucontrol->value.integer.value[0] == es9218_hifi_control)
        return 1;

    es9218_hifi_control = ucontrol->value.integer.value[0];

    es9218->hifi_status=es9218_hifi_control;

    es9218_mode_set(es9218);

    return 0;
}
static int es9218_bit_set(struct snd_kcontrol *kcontrol,
        struct snd_ctl_elem_value *ucontrol)
{
    struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
    struct es9218_data *es9218 = snd_soc_component_get_drvdata(component);
    int bits=0;
    int reg_val=0;
    unsigned int length=0;
    pr_debug("%s: ucontrol->value.integer.value[0]=%ld\n ",
            __func__, ucontrol->value.integer.value[0]);

    if( es9218_hifi_control != ES9218_STATUS_HIFI)
        return 0;
	
    bits = ucontrol->value.integer.value[0];
    reg_val=es9218_read_byte(es9218,ES9218_CONTROL_REG01);
    pr_info("%s read 01-reg data=0x%x\n",__func__,reg_val);

    reg_val &= ES9218_SERIAL_LENGTH_MASK;

    switch(bits)
    {
         case 0:
            length=ES9218_SERIAL_LENGTH_16;
         break;
         case 1:
             length=ES9218_SERIAL_LENGTH_24;
         break;
         case 2:
            length=ES9218_SERIAL_LENGTH_32;
         break;
         default:
            length=ES9218_SERIAL_LENGTH_16;
         break;
    }
    
    reg_val |= length;

    es9218_write_byte(es9218,ES9218_CONTROL_REG01,reg_val);

    /*ZSQ add: change es9038q2m_init_register after hifi set*/ 
    if ( es9038_type == ES9038_Q2M){
       es9038q2m_init_register[ES9218_CONTROL_REG01][1] &= ES9218_SERIAL_LENGTH_MASK;
       es9038q2m_init_register[ES9218_CONTROL_REG01][1] |= length;
    }else{
       es9038k2m_init_register[ES9218_CONTROL_REG01][1] &= ES9218_SERIAL_LENGTH_MASK;
       es9038k2m_init_register[ES9218_CONTROL_REG01][1] |= length;
	}
    
    pr_info("%s write 01-reg data=0x%x\n",__func__,reg_val);
    return 0;
}
static int es9218_bit_get(struct snd_kcontrol *kcontrol,
        struct snd_ctl_elem_value *ucontrol)
{
    struct snd_soc_component *component =snd_soc_kcontrol_component(kcontrol);
    struct es9218_data *es9218 = snd_soc_component_get_drvdata(component);
    int reg_value=0;
	if( es9218_hifi_control == ES9218_STATUS_HIFI){    
        reg_value=es9218_read_byte(es9218,ES9218_CONTROL_REG01);
        if (reg_value >= 0) {
            reg_value &= (~ES9218_SERIAL_LENGTH_MASK);
            reg_value >>= 6;
        } else {
            reg_value = 2;
        }
	}
    pr_debug("%s: es9218_hifi_bit=%d\n", __func__, reg_value);
    ucontrol->value.integer.value[0] = reg_value;
    return 0;
}

static int es9218_master_set(struct snd_kcontrol *kcontrol,
        struct snd_ctl_elem_value *ucontrol)
{
    struct snd_soc_component *component =snd_soc_kcontrol_component(kcontrol);
    struct es9218_data *es9218 = snd_soc_component_get_drvdata(component);
    int master=0;
    int reg_val=0;
    pr_debug("%s: ucontrol->value.integer.value[0]=%ld\n ",
            __func__, ucontrol->value.integer.value[0]);
    
    master = ucontrol->value.integer.value[0];
    reg_val=es9218_read_byte(es9218,ES9218_CONTROL_REG10);
    pr_info("%s read 01-reg data=0x%x\n",__func__,reg_val);

    if (master) {
        reg_val |= 0x80;
	/*ZSQ add: change es9218_init_register after hifi set*/ 
       if ( es9038_type == ES9038_Q2M)
	      es9038q2m_init_register[ES9218_CONTROL_REG10][1] |= 0x80;
	   else
	   	  es9038k2m_init_register[ES9218_CONTROL_REG10][1] |= 0x80;
    } else {
        reg_val &= 0x7f;
        /*ZSQ add: change es9038q2m_init_register after hifi set*/
		if ( es9038_type == ES9038_Q2M)
           es9038q2m_init_register[ES9218_CONTROL_REG10][1] &= 0x7f;
		else
		   es9038k2m_init_register[ES9218_CONTROL_REG10][1] &= 0x7f;
    }

    es9218_write_byte(es9218,ES9218_CONTROL_REG10,reg_val);

    pr_info("%s write 10-reg data=0x%x\n",__func__,reg_val);
    return 0;
}
static int es9218_master_get(struct snd_kcontrol *kcontrol,
        struct snd_ctl_elem_value *ucontrol)
{
    struct snd_soc_component *component =snd_soc_kcontrol_component(kcontrol);
    struct es9218_data *es9218 = snd_soc_component_get_drvdata(component);

    int reg_value=0;
	if( es9218_hifi_control == ES9218_STATUS_HIFI){
        reg_value=es9218_read_byte(es9218,ES9218_CONTROL_REG10);
        if (reg_value >= 0)  {
            reg_value &= 0x80;
        }  else {
            reg_value = 0;
        }
	}
    pr_debug("%s: es9218_hifi_master_status=%d\n", __func__, reg_value);
    ucontrol->value.integer.value[0] = reg_value ? 1 : 0;
    return 0;
}
static int es9218_clk_div_set(struct snd_kcontrol *kcontrol,
        struct snd_ctl_elem_value *ucontrol)
{
    struct snd_soc_component *component =snd_soc_kcontrol_component(kcontrol);
    struct es9218_data *es9218 = snd_soc_component_get_drvdata(component);
    int master=0;
    int reg_val=0;
    pr_debug("%s: ucontrol->value.integer.value[0]=%ld\n ",
            __func__, ucontrol->value.integer.value[0]);
    
    master = ucontrol->value.integer.value[0];
    reg_val=es9218_read_byte(es9218,ES9218_CONTROL_REG10);
    pr_info("%s read 01-reg data=0x%x\n",__func__,reg_val);

    reg_val &= 0x9F;
    reg_val |= master<<5;
    es9218_write_byte(es9218,ES9218_CONTROL_REG10,reg_val);

    /*ZSQ add: change es9218_init_register after hifi set*/ 
	if ( es9038_type == ES9038_Q2M){
       es9038q2m_init_register[ES9218_CONTROL_REG10][1] &= 0x9F;
       es9038q2m_init_register[ES9218_CONTROL_REG10][1] |= master<<5;
	}else{
       es9038k2m_init_register[ES9218_CONTROL_REG10][1] &= 0x9F;
       es9038k2m_init_register[ES9218_CONTROL_REG10][1] |= master<<5;
	}
		

    pr_info("%s write 10-reg data=0x%x\n",__func__,reg_val);
    return 0;
}

static int es9218_clk_div_get(struct snd_kcontrol *kcontrol,
        struct snd_ctl_elem_value *ucontrol)
{
    struct snd_soc_component *component =snd_soc_kcontrol_component(kcontrol);
    struct es9218_data *es9218 = snd_soc_component_get_drvdata(component);

    unsigned int reg_value=0;
	if( es9218_hifi_control == ES9218_STATUS_HIFI){
        reg_value=es9218_read_byte(es9218,ES9218_CONTROL_REG10);
        reg_value &= 0x60;
        reg_value >>=5;
    }
    pr_debug("%s: es9218_hifi_clk_div=%d\n", __func__, reg_value);
    ucontrol->value.integer.value[0] = reg_value;
    return 0;
}


static const struct soc_enum es9218_snd_enum[] = {
    SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(hifi_function), hifi_function),
};
static const struct soc_enum es9218_bit_enum[] = {
    SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(hifi_bit), hifi_bit),
};

static const struct soc_enum es9218_master_enum[] = {
    SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(hifi_master), hifi_master),
};
static const struct soc_enum es9218_clk_div_enum[] = {
    SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(hifi_clk_div), hifi_clk_div),
};



static struct snd_kcontrol_new es9218_controls[] = {
    SOC_ENUM_EXT("es9218_hifi_switch", es9218_snd_enum[0],
            es9218_hifi_get, es9218_hifi_set),
};
static struct snd_kcontrol_new es9218_set_bits[] = {
    SOC_ENUM_EXT("es9218_set_bits", es9218_bit_enum[0],
            es9218_bit_get, es9218_bit_set),
};

static struct snd_kcontrol_new es9218_set_master[] = {
    SOC_ENUM_EXT("es9218_set_master", es9218_master_enum[0],
            es9218_master_get, es9218_master_set),
};

static struct snd_kcontrol_new es9218_clk_div[] = {
    SOC_ENUM_EXT("es9218_clk_div", es9218_clk_div_enum[0],
            es9218_clk_div_get, es9218_clk_div_set),
};


static void es9218_add_codec_controls(struct snd_soc_component *compoent)
{
    pr_info("%s enter\n", __func__);
    snd_soc_add_component_controls(compoent, es9218_controls,
            ARRAY_SIZE(es9218_controls));

    snd_soc_add_component_controls(compoent, &es9218_volume,1);

    snd_soc_add_component_controls(compoent,es9218_set_bits,ARRAY_SIZE(es9218_set_bits));
    
    snd_soc_add_component_controls(compoent,es9218_set_master,ARRAY_SIZE(es9218_set_master));

    snd_soc_add_component_controls(compoent,es9218_clk_div,ARRAY_SIZE(es9218_clk_div));
}
#endif

/*****************************************************
 *
 * codec driver
 *
 *****************************************************/
static int es9218_probe(struct snd_soc_component *component)
{ 
    struct es9218_data *es9218 = snd_soc_component_get_drvdata(component);
    int ret = 0;
     pr_info("%s enter\n", __func__);
     if(es9218)
     { //es9218->codec = codec;
	     pr_info("%s get es9218_data\n",__func__);
     }else{
     	 return -1;
     }
      
   //add kcontrol
    es9218_add_codec_controls(component);
    if (es9218->dev->of_node)
        dev_set_name(es9218->dev, "%s", "hifi-es9218");

    pr_info("%s exit\n", __func__);
    return ret;
}

static void es9218_remove(struct snd_soc_component *component)
{
    pr_info("%s enter\n", __func__);      
}

static unsigned int es9218_codec_read(struct snd_soc_component *component,unsigned int reg)
{
    struct es9218_data *es9218=snd_soc_component_get_drvdata(component);
    int value =0;
    pr_debug("%s:enter \n",__func__);

    if(reg <= ES9218_REG_MAX){
        value=es9218_read_byte(es9218,reg);
    }else{
        pr_debug("%s:Register 0x%x NO read access\n",__func__,reg);
        return -1;
    }
    return value;
}
static int es9218_codec_write(struct snd_soc_component *component,unsigned int reg,unsigned int value)
{
    int ret ;
    struct es9218_data *es9218=snd_soc_component_get_drvdata(component);
    pr_debug("%s:enter ,reg is 0x%x value is 0x%x\n",__func__,reg,value);

    if(reg < ES9218_REG_MAX){
        ret=es9218_write_byte(es9218,reg,value);
        return ret;
    }else{
        pr_debug("%s: Register 0x%x NO write access \n",__func__,reg);
    }

    return -1;
}

static struct snd_soc_component_driver soc_codec_dev_es9218 = {
    .probe = es9218_probe,
    .remove = es9218_remove,
    .read = es9218_codec_read,
    .write= es9218_codec_write,
};

static int es9218_power_enable(struct es9218_data *edata, int enable)
{
    pr_debug("%s Power supply setting enable:%d.\n",__func__, enable);

	if (enable) {
		gpio_set_value_cansleep(edata->rst_gpio,1);
		if (!IS_ERR_OR_NULL(edata->hifi_mclk)) {
		    clk_prepare_enable(edata->hifi_mclk);
			pr_info("%s: MCLK enable.\n", __func__);
		}
		usleep_range(5000, 5500);
	} else {
		es9218_write_byte(edata, 32, 0x00);
		usleep_range(10000, 15000);		
		gpio_set_value_cansleep(edata->rst_gpio,1); // hmct test
		/*ZSQ add using bypass setting to avoid earphone detect problem*/	
//		gpio_set_value_cansleep(edata->hifi_sel_gpio,1);
//		gpio_set_value_cansleep(edata->hph_sel_gpio,0);
		usleep_range(5000, 5500);		
        /*ZSQ add oscillator gpio control */
      	gpio_set_value_cansleep(edata->osci_gpio,0);
		gpio_set_value_cansleep(edata->dac_pwr_gpio,0);
		gpio_set_value_cansleep(edata->pa_pwr_gpio,0);
	}
	return 0;
}

static void es9218_reset(struct es9218_data *edata)
{
	pr_info("%s es9218 reset.\n",__func__);
	/* gpio_direction_output(edata->rst_gpio, 0); */
	usleep_range(5000, 5500);
	//gpio_direction_output(edata->rst_gpio, 1);
	if(gpio_is_valid(edata->rst_gpio))
	{
	  gpio_set_value_cansleep(edata->rst_gpio,0);
	  pr_info("%s es9218 set rst gpio to high.\n",__func__);
	  usleep_range(5000, 5500);
	  gpio_set_value_cansleep(edata->rst_gpio,1);
	  usleep_range(5000, 5500);
	}
}

int es9218_dsd_set_volume(int index)
{
	int ret = 0;
	int reg_val = 0;

	pr_info("%s start.\n", __func__);

	if ((index > 32) || (index < 0))
		pr_err("%s volume index error:%d\n",
			__func__, index);

	if (pes9218_data->on) {
		ret = es9218_write_byte(pes9218_data, 15, dsd_volume[index][1]);
		if (ret < 1)
			pr_err("%s set volume reg#15 error\n",__func__);

		ret = es9218_write_byte(pes9218_data, 16, dsd_volume[index][1]);
		if (ret < 1)
			pr_err("%s set volume reg#16 error\n",__func__);

		reg_val = es9218_read_byte(pes9218_data, 15);
		pr_info("%s volume regsiter #15:0x%x.\n",__func__, reg_val);

		reg_val = es9218_read_byte(pes9218_data, 16);
		pr_info("%s volume regsiter #16:0x%x.\n",__func__, reg_val);
	} else {
		pr_err("%s es9218 not open\n", __func__);
	}

	return ret;
}

static int es9218_setting_mode_params(struct es9218_params *params)
{
	int ret = 0;

	pr_info("%s params->mode %d\n",	__func__, params->mode);
	if (params->mode < 0) {
		pr_err("%s no params to set\n",	__func__);
		return 0;
	}
	ret = params->mode;
	return ret;
}

int es9218_setting_special_params(struct es9218_data *edata,
	struct es9218_params *params)
{
	int i = 0;
	struct es9218_reg_peer *peer;

	if (params->size <= 0 || params->peer == NULL) {
		pr_warn("%s no params to set\n", __func__);
		return 0;
	}

	for (i = 0; i < params->size; i++) {
		peer = params->peer + i;
		if (peer) {
			pr_debug("%s write value 0x%x to addr 0x%x.\n",__func__, peer->val, peer->addr);
			es9218_write_byte(edata, peer->addr, peer->val);
		} else
			pr_err("%s peer is NULL, ignore writing.\n",__func__);
	}

	return 0;
}

static int do_reg_write_check(struct es9218_data *edata, u8 reg, u8 val)
{
	u8 reg_val = 0;
	int retry = 1;
	int ret = 0;
	char logbuf[100];
	int n = 0;

	pr_info("%s: entry.\n", __func__);

	reg_val = es9218_read_byte(edata, reg);
	while (val != reg_val && retry > 0) {
		es9218_write_byte(edata, reg, val);
		reg_val = es9218_read_byte(edata, reg);
		retry--;
	}

	if (retry == 1) {
		ret = 0;
	} else if (retry >= 0 && val == reg_val) {
		n = scnprintf(logbuf, sizeof(logbuf),
			"Write 0x%2x to 0x%2x success, retry %d times\n",
			val, reg, 5 - retry);
#ifdef CONFIG_FMEA
		fmea_notify("es9218", logbuf);
#endif
		ret = 0;
	} else if (!retry) {
		n = scnprintf(logbuf, sizeof(logbuf),
			"Write 0x%2x to 0x%2x failed, retry 5 times\n",
			val, reg);
#ifdef CONFIG_FMEA
		fmea_notify("es9218", logbuf);
#endif
		ret = -1;
	}

	pr_info("%s: exit, n: %d.\n", __func__, reg_val);

	return ret;
}

static int es9218_one_enable(struct es9218_data *edata,
	struct audio_params *params, bool enable)
{
	u8 reg_val;
	int para_num = sizeof(dac_parameters) / 2;
	int i, clk_div;
	int ret = 0;
	int hifi_mode_status=-1;

	if (edata->client == NULL) {
		pr_err("%s: client is NULL.\n ", __func__);
		return -EFAULT;
	}

	if (!params) {
		pr_err("%s:params is NULL\n", __func__);
		return -EINVAL;
	}

	memcpy(&edata->params, params, sizeof(struct audio_params));

	if (params->private_params)
		hifi_mode_status = es9218_setting_mode_params(params->private_params);

	pr_info("%s: enable %d, pcm_format 0x%x, params->i2s_format 0x%x,"
			" i2s_rate: %d, dac_mode: %d.\n", __func__, enable,
			params->pcm_format, params->i2s_format,
			params->rate, hifi_mode_status);

	mutex_lock(&edata->lock);

	if (enable) {/* power on */
		if (hifi_mode_status == 1) { /* hifi mode */
			pr_info("%s: hifi mode set.\n", __func__);

			if ((params->i2s_format & SND_SOC_DAIFMT_MASTER_MASK)
				== SND_SOC_DAIFMT_CBM_CFM) { /* codec master mode */
			} else { /* codec slave mode */
				clk_div = 19200000 / params->rate / 64;

				pr_info("%s: clk_div: %d.\n", __func__, clk_div);

				switch (clk_div) {
				case 1:
					es9218_write_byte(edata, 128, 0x8c);
					es9218_write_byte(edata, 129, 0x7d);
					es9218_write_byte(edata, 130, 0x00);
					es9218_write_byte(edata, 131, 0x03);
					es9218_write_byte(edata, 132, 0x02);
					es9218_write_byte(edata, 133, 0x3c);
					usleep_range(3000, 3200);
					es9218_write_byte(edata, 128, 0xcc);
					break;
				case 3:
					es9218_write_byte(edata, 0, 0x00);
					break;
				case 6:
					es9218_write_byte(edata, 0, 0x10);
					break;
				default:
					break;
				}
			}
			/* setting es9118 initialization */
			for (i = 0; i < para_num; i++) {
				es9218_write_byte(edata,dac_parameters[i][0], dac_parameters[i][1]);
				ret = do_reg_write_check(edata,dac_parameters[i][0], dac_parameters[i][1]);
				if (ret < 0)
					goto end;
			}

			usleep_range(5000, 5200);
			es9218_write_byte(edata, 32, 0x02);

			pr_info("%s hifi-initialization-done %d\n",
				__func__, hifi_mode_status);
			edata->on = true;
			ret = 0;
		} else if (hifi_mode_status == 2) { /* dsd mode */
			pr_info("%s: dsd mode set.\n", __func__);

			/* Set hifi_mclk & pll */
			es9218_write_byte(edata, 128, 0x8c);
			es9218_write_byte(edata, 129, 0x7d);
			es9218_write_byte(edata, 130, 0x00);
			es9218_write_byte(edata, 131, 0x03);
			es9218_write_byte(edata, 132, 0x02);
			es9218_write_byte(edata, 133, 0x3c);
			usleep_range(5000, 5200);
			if (!IS_ERR_OR_NULL(edata->hifi_mclk)) {
			    clk_prepare_enable(edata->hifi_mclk);
				pr_info("%s: MCLK enable.\n", __func__);
			}
			usleep_range(5000, 5200);
			es9218_write_byte(edata, 128, 0xcc);

			for (i = 0; i < para_num; i++) {
				es9218_write_byte(edata,dac_parameters[i][0], dac_parameters[i][1]);
				ret = do_reg_write_check(edata,dac_parameters[i][0], dac_parameters[i][1]);
				if (ret < 0)
					goto end;
			}

			reg_val = es9218_read_byte(edata, 2);
			reg_val &= 0x34;
			reg_val |= 4;
			ret = es9218_write_byte(edata, 2, reg_val);
			reg_val = es9218_read_byte(edata, 1);
			reg_val = reg_val & 0xfc;
			es9218_write_byte(edata, 1, reg_val);
			reg_val = es9218_read_byte(edata, 6);
			reg_val |= 0x1 << 3 ;
			es9218_write_byte(edata, 6, reg_val);

			es9218_write_byte(edata, 32, 0x02);

			if (params->private_params)
				es9218_setting_special_params(edata,(params->private_params));

			edata->on = true;
			ret = 0;

		} else if (hifi_mode_status == 3) { /* low power mode */
			pr_info("%s: low power mode set.\n", __func__);

			if (!IS_ERR_OR_NULL(edata->hifi_mclk)) {
			    clk_prepare_enable(edata->hifi_mclk);
				pr_info("%s: MCLK enable.\n", __func__);
			}
			usleep_range(5000, 5200);

			es9218_write_byte(edata, 0, 0x10);
			for (i = 0; i < para_num; i++) {
				es9218_write_byte(edata,dac_parameters[i][0], dac_parameters[i][1]);
				ret = do_reg_write_check(edata,dac_parameters[i][0], dac_parameters[i][1]);
				if (ret < 0)
					goto end;
			    es9218_write_byte(edata, 29, 0xc0);
			    es9218_write_byte(edata, 32, 0x02);
			}

			edata->on = true;
			ret = 0;
		} else if (hifi_mode_status == 4) { /* platform hp mode */
			pr_info("%s: platform hp mode set.\n", __func__);
			usleep_range(3000, 3200);
		    es9218_write_byte(edata, 38, 0x30);
		    usleep_range(3000, 3200);
		    gpio_direction_output(edata->rst_gpio, 0);
		    usleep_range(3000, 3200);
		} else { /* platform hp mode exit for debug by --. */
			pr_info("%s: platform mode exit.\n", __func__);
			gpio_direction_output(edata->rst_gpio, 1);
			usleep_range(3000, 3200);
			es9218_write_byte(edata, 32, 0x01);
			es9218_write_byte(edata, 38, 0x70);
			usleep_range(3000, 3200);
			es9218_write_byte(edata, 38, 0x00);
			usleep_range(3000, 3200);
			es9218_write_byte(edata, 32, 0x00);
			es9218_power_enable(edata, 0);
			edata->on = false;
		    ret = 0;
		}
	} else { /* power down */
		/* mute dac first */
		/* usleep_range(10000, 12000); */
		es9218_power_enable(edata, 0);
		usleep_range(10000, 12000);
		edata->on = false;
		ret = 0;
	}

end:
	mutex_unlock(&edata->lock);
	return ret;
}

int es9218_enable(struct audio_params *params, bool enable)
{
	if (pes9218_data) {
		/* power up and reset */
		if (enable) {
			es9218_power_enable(pes9218_data, 1);
			usleep_range(5000, 5200);
			es9218_reset(pes9218_data);
			usleep_range(3000, 3200);
		}
		/* enable master */
		es9218_one_enable(pes9218_data, params, enable);
	}

	return 0;
}

int es9218_one_mute(struct es9218_data *edata, int mute)
{
	int reg_val = 0;
      struct es9218_data *pdata=edata;
	pr_info("%s: mute: %d.\n", __func__, mute);

	if (!pdata->on) {
		pr_warn("%s() es9038(addr 0x%.2x) not enabled, exit.\n",
			__func__, pdata->client->addr);
		return 0;
	}

	reg_val = es9218_read_byte(pdata, ES9218_CONTROL_REG07);

	if (mute) {
		/* es9038_i2c_write_byte(edata, 0x48, 14, 0x0a); */
		reg_val = reg_val | 0x1;
		es9218_write_byte(pdata, ES9218_CONTROL_REG07, reg_val);
		es9218_write_byte(pdata, ES9218_CONTROL_REG14, 0x02);
		/* -- add for POP,hifi 350ms ramp down,dsd 100ms ramp down */
		usleep_range(250000, 260000);
	} else {
		reg_val = reg_val & 0xfe;
		/* -- from 0x89 to 0x8a */
		es9218_write_byte(pdata, ES9218_CONTROL_REG14, 0x8a);
		es9218_write_byte(pdata, ES9218_CONTROL_REG07, (unsigned char)reg_val);
	}
	/* es9038_i2c_write_byte(edata, 0x48, 7, reg_val); */

	return 0;
}

int es9218_mute(int mute)
{
	if (pes9218_data)
		es9218_one_mute(pes9218_data, (mute & 1));

	if (mute)
		msleep(45); /* reg_e d4~d0, 8 means 43ms */

	return 0;
}

#ifdef CONFIG_DEBUG_FS
static int es9218_debug_open(struct inode *inode, struct file *file)
{
    pr_info("%s: entry.\n", __func__);

	file->private_data = inode->i_private;
	current_debugfs = (struct es9218_data *)file->private_data;

	return 0;
}

static int es9218_debug_release (struct inode *inode, struct file *filep)
{
	int ret = 0;

    pr_info("%s: entry.\n", __func__);
    current_debugfs = NULL;
    filep->private_data = NULL;
	return ret;
}

static ssize_t es9218_debug_write(struct file *filp,
	const char __user *ubuf, size_t cnt, loff_t *ppos)
{
	int ret = 0;
	unsigned int kbuf[2];
	char *temp;

	temp = kmalloc(cnt, GFP_KERNEL);
	if (!temp) {
		return -ENOMEM;
	}

	ret = copy_from_user(temp, (void __user *)ubuf, cnt);
	ret = sscanf(temp, "%x %x", &kbuf[0], &kbuf[1]);
	if (!ret) {
		kfree(temp);
		return -EFAULT;
	}

	pr_info("%s: kbuf[0]: 0x%x, kbuf[1]: 0x%x cnt: %lu.\n",
		__func__, kbuf[0], kbuf[1], cnt);

	if (!current_debugfs) {
		pr_err("%s() current_debugfs is NULL\n", __func__);
		return -ENOMEM;
	}

	/* if dac is not on, return */
	if (!current_debugfs->on) {
		pr_info("%s() sorry, dac(addr 0x%.2x) is off!\n",
			__func__, current_debugfs->client->addr);
		return 0;
	}

	/* if dac is on, write registers */
	es9218_write_byte(current_debugfs, kbuf[0], kbuf[1]);
	do_reg_write_check(current_debugfs,kbuf[0], kbuf[1]);
	return cnt;
}

static ssize_t es9218_debug_read(struct file *file, char __user *buf,
				     size_t count, loff_t *pos)
{
	int i;
	const int size = 1024;
	u8 data;
	char buffer[size];
	int n = 0;

	if (!current_debugfs) {
		pr_err("%s() current_debugfs is NULL\n",
			__func__);
		return -ENOMEM;
	}
       current_debugfs->on=true;
	/* if dac is not on, return */
	if (!current_debugfs->on) {
		n = scnprintf(buffer, size, "Sorry, dac(addr 0x%.2x) is off!\n",
				current_debugfs->client->addr);

		buffer[n] = 0;
		pr_info("%s(): %s", __func__, buffer);
		return simple_read_from_buffer(buf, count, pos, buffer, n);
	}

	pr_info("%s:---caught es9218 reg start---\n",__func__);
	/* if dac is on, dump registers */
	if ( es9038_type == ES9038_Q2M){
	   for (i = 0; i < sizeof(es9038q2m_init_register)/2; i++) {
		   data = es9218_read_byte(current_debugfs,es9038q2m_init_register[i][0]);
		   n += scnprintf(buffer+n, size-n, "reg{0x%x}: 0x%x\n",es9038q2m_init_register[i][0], data);
		   pr_info("%s:es9218 reg[%d]: 0x%x.\n",__func__, es9038q2m_init_register[i][0], data);
	   }
	}else{
       for (i = 0; i < sizeof(es9038k2m_init_register)/2; i++) {
		   data = es9218_read_byte(current_debugfs,es9038k2m_init_register[i][0]);
		   n += scnprintf(buffer+n, size-n, "reg{0x%x}: 0x%x\n",es9038k2m_init_register[i][0], data);
		   pr_info("%s:es9218 reg[%d]: 0x%x.\n",__func__, es9038k2m_init_register[i][0], data);
	   }
	}
	for (i = 64; i <77; i++) {
		data = es9218_read_byte(current_debugfs,i);
		pr_info("%s:es9218 reg[%d]: 0x%x.\n",__func__, i, data);
      }
	return simple_read_from_buffer(buf, count, pos, buffer, n);
}

static struct file_operations es9218_debugfs_fops = {
	.open = es9218_debug_open,
	.read = es9218_debug_read,
	.write = es9218_debug_write,
	.release = es9218_debug_release,
};

static ssize_t es9218_debug_i2c_write(struct file *filp,
	const char __user *ubuf, size_t cnt, loff_t *ppos)
{
	int ret = 0;
	unsigned int kbuf[2];
	char *temp;

	temp = kmalloc(cnt, GFP_KERNEL);
	if (!temp) {
		return -ENOMEM;
	}

	ret = copy_from_user(temp, (void __user *)ubuf, cnt);
	ret = sscanf(temp, "%x %x", &kbuf[0], &kbuf[1]);
	if (!ret) {
		kfree(temp);
		return -EFAULT;
	}

	pr_info("%s: kbuf[0]: 0x%x, kbuf[1]: 0x%x cnt: %lu.\n",
		__func__, kbuf[0], kbuf[1], cnt);

	if (!current_debugfs) {
		pr_err("%s() current_debugfs is NULL\n", __func__);
		return -ENOMEM;
	}

	/* for debug by -- */
	if (kbuf[0] || kbuf[1]) {
		pr_info("%s: reset pin is set high.\n",
			__func__);
		gpio_direction_output(pes9218_data->rst_gpio, 1);
	} else {
		pr_info("%s: reset pin is set low.\n",
			__func__);
		gpio_direction_output(pes9218_data->rst_gpio, 0);
	}

	return cnt;
}

static ssize_t es9218_debug_i2c_read(struct file *file, char __user *buf,
	size_t count, loff_t *pos)
{
	const int size = 512;
	char buffer[size];
	int n = 0;

	pr_info("[HiFi-%s]: i2c read enter.\n", __func__);

	if (!current_debugfs) {
		pr_err("%s() current_debugfs is NULL\n",
			__func__);
		return -ENOMEM;
	}

	n += scnprintf(buffer+n, size-n, "HiFi-0x%x %s\n",
		current_debugfs->client->addr, 0x48 ? "OK" : "ERROR");

	buffer[n] = 0;

	return simple_read_from_buffer(buf, count, pos, buffer, n);
}
static ssize_t es9218_debug_chipid_read(struct file *file, char __user *buf,
	size_t count, loff_t *pos)
{
	const int size = 512;
	char buffer[size];
	int n = 0;

	pr_info("[HiFi-%s]:  read chipid enter.\n", __func__);

	 n += scnprintf(buffer+n, size-n, "HiFi-chipid=0x%x\n",Chipid);

	buffer[n] = 0;

	return simple_read_from_buffer(buf, count, pos, buffer, n);
}

static ssize_t es9218_debug_chipid_write(struct file *filp,
	const char __user *ubuf, size_t cnt, loff_t *ppos)
{
	
	return 0;
}

static struct file_operations es9218_i2c_debugfs_fops = {
	.open = es9218_debug_open,
	.read = es9218_debug_i2c_read,
	.write = es9218_debug_i2c_write,
};
static struct file_operations es9218_chipid_debugfs_fops = {
	.open = es9218_debug_open,
	.read = es9218_debug_chipid_read,
	.write = es9218_debug_chipid_write,
};
#endif
static void es9218_chip_init(struct es9218_data *edata)
{
    int i=0;
    int ret=0;
    int param_num=0;
    if ( es9038_type == ES9038_Q2M){
		param_num=sizeof(es9038q2m_init_register) / 2;
	    pr_info("%s param_num=%d\n",__func__,param_num);
		/* setting es9118 initialization */
	    for (i = 0; i < param_num; i++) {
			es9218_write_byte(edata,es9038q2m_init_register[i][0], es9038q2m_init_register[i][1]);
			ret = do_reg_write_check(edata,es9038q2m_init_register[i][0], es9038q2m_init_register[i][1]);
			if (ret < 0){
			    pr_err("%s check reg write error \n",__func__);	
			    return ;
			}
	    }
     }else{
        param_num=sizeof(es9038k2m_init_register) / 2;
	    pr_info("%s param_num=%d\n",__func__,param_num);
		/* setting es9118 initialization */
	    for (i = 0; i < param_num; i++) {
			es9218_write_byte(edata,es9038k2m_init_register[i][0], es9038k2m_init_register[i][1]);
			ret = do_reg_write_check(edata,es9038k2m_init_register[i][0], es9038k2m_init_register[i][1]);
			if (ret < 0){
			    pr_err("%s check reg write error \n",__func__);	
			    return ;
			}
	    }
	 }
     
}

static int es9218_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	struct snd_soc_dai_driver *dai;
	int ret = 0;
	u8 chipid = 0x0;
	int retry = 5;
#ifdef CONFIG_DEBUG_FS
	struct dentry *es9218_debugfs;
#endif
	struct es9218_data *edata;

    pr_info("%s: entry.\n", __func__);
    edata = kzalloc(sizeof(struct es9218_data), GFP_KERNEL);

	if (!edata) {
		dev_err(&client->dev, "%s kzalloc failed\n", __func__);
		return -ENOMEM;
	}

	if (client->dev.of_node) {
		/*dac reset gpio and sel gpio request*/
		edata->rst_gpio = of_get_named_gpio(client->dev.of_node,"reset-gpio", 0);
		if (gpio_is_valid(edata->rst_gpio)) {
			ret = devm_gpio_request_one(&client->dev, edata->rst_gpio,GPIOF_OUT_INIT_LOW, "es9218_rst");
			if (ret){
				  pr_err("%s: rst request failed\n", __func__);
				  ret =-1;
				  goto err_regmap;
			}else{
			      pr_info("%s: rst request ok\n", __func__);
			}
		}else{
		      pr_err("%s: rst gpio is not valid\n", __func__);
		      goto err_regmap;	  
		}
		
		edata->hifi_sel_gpio = of_get_named_gpio(client->dev.of_node,"sel-gpio", 0);
		if (gpio_is_valid(edata->hifi_sel_gpio)) {
			ret = devm_gpio_request_one(&client->dev, edata->hifi_sel_gpio,GPIOF_OUT_INIT_HIGH, "es9218_sel");
			if (ret){
				  pr_err("%s: sel gpio request failed\n", __func__);
				  ret =-1;
				  goto err_sel_gpio_request;
			}else{
			      pr_info("%s: sel gpio request ok\n", __func__);
			}
		}else{
		      pr_err("%s: sel gpio is not valid\n", __func__);
		      goto err_sel_gpio_request;  
		}

		edata->hph_sel_gpio = of_get_named_gpio(client->dev.of_node,"sel2-gpio", 0);
		if (gpio_is_valid(edata->hph_sel_gpio)) {
			ret = devm_gpio_request_one(&client->dev, edata->hph_sel_gpio,GPIOF_OUT_INIT_HIGH, "es9218_sel2");
			if (ret){
				  pr_err("%s: sel gpio request failed\n", __func__);
				  ret =-1;
				  goto err_sel_gpio_request;
			}else{
			      pr_info("%s: sel gpio request ok\n", __func__);
			}
		}else{
		      pr_err("%s: sel gpio is not valid\n", __func__);
		      goto err_sel_gpio_request;  
		}

		/*ZSQ add oscillator gpio control */
		edata->osci_gpio = of_get_named_gpio(client->dev.of_node,"osci-gpio", 0);
		if (gpio_is_valid(edata->osci_gpio)) {
			ret = devm_gpio_request_one(&client->dev, edata->osci_gpio,GPIOF_OUT_INIT_HIGH, "es9218_osci");
			if (ret){
				  pr_err("%s: osci gpio request failed\n", __func__);
				  ret =-1;
				  goto err_osci_gpio_request;
			}else
			{
			      pr_info("%s: osci gpio request ok\n", __func__);
			}
		}else
		{
		      pr_err("%s: osci gpio is not valid\n", __func__);
		      goto err_osci_gpio_request; 	  
		}

        edata->dac_pwr_gpio = of_get_named_gpio(client->dev.of_node,"dac-pwr-gpio", 0);
		if (gpio_is_valid(edata->dac_pwr_gpio)) {
			ret = devm_gpio_request_one(&client->dev, edata->dac_pwr_gpio,GPIOF_OUT_INIT_HIGH, "es9218_dac_pwr");
			if (ret){
				  pr_err("%s: dac_pwr gpio request failed\n", __func__);
				  ret =-1;
				  goto err_osci_gpio_request;
			}else
			{
			      pr_info("%s: dac_pwr gpio request ok\n", __func__);
			}
		}else
		{
		      pr_err("%s: osci gpio is not valid\n", __func__);
		      goto err_osci_gpio_request; 	  
		}

		edata->pa_pwr_gpio = of_get_named_gpio(client->dev.of_node,"pa-pwr-gpio", 0);
		if (gpio_is_valid(edata->pa_pwr_gpio)) {
			ret = devm_gpio_request_one(&client->dev, edata->pa_pwr_gpio,GPIOF_OUT_INIT_HIGH, "es9218_pa_pwr");
			if (ret){
				  pr_err("%s: pa_pwr gpio request failed\n", __func__);
				  ret =-1;
				  goto err_osci_gpio_request;
			}else
			{
			      pr_info("%s: pa_pwr gpio request ok\n", __func__);
			}
		}else
		{
		      pr_err("%s: pa_pwr gpio is not valid\n", __func__);
		      goto err_osci_gpio_request; 	  
		}

		pes9218_data = edata;
              i2c_set_clientdata(client, edata);
		/* read dac params for each impedance level */
    }

	edata->client = client;
	edata->driver_name = ES9218_DEV_NAME;
	edata->dev = &client->dev;
	//edata->fun = function;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "%s i2c check funtion error \n",__func__);
		goto err_es9218_data;
	}
#ifdef ES9218_I2C_REMAP
/* es9218 regmap */
      edata->regmap = devm_regmap_init_i2c(client, &es9218_regmap);
      if (IS_ERR(edata->regmap)) {
	   ret = PTR_ERR(edata->regmap);
	   dev_err(&client->dev, "%s: failed to allocate register map: %d\n", __func__, ret);
	goto err_regmap;
}
#endif
	es9218_power_enable(edata, 1);
	if (!IS_ERR_OR_NULL(edata->hifi_dac_regulator)) {
		clk_prepare_enable(edata->hifi_mclk);
		pr_info("%s: MCLK enable.\n", __func__);
	}
	es9218_reset(edata);

	chipid = es9218_read_byte(edata, ES9218_ID_REG);
	
	while (retry--) {
		if (ES9038Q2M_CHIP_ID ==(chipid & 0xFC)) {
			es9038_type = ES9038_Q2M;
			pr_info("%s:chip id correct: 0x%x\n", __func__, chipid);
			break;			
		}else if (ES9038K2M_CHIP_ID ==(chipid & 0xFC)) {
		    es9038_type = ES9038_K2M;
			pr_info("%s:chip id correct: 0x%x\n", __func__, chipid);
			break;			
		}else {
		    msleep(5);
			chipid = es9218_read_byte(edata, ES9218_ID_REG);
			pr_err("%s:chip id: 0x%x\n", __func__, chipid);		      
		}
	}

	if (ES9038Q2M_CHIP_ID !=(chipid & 0xFC)&&
		ES9038K2M_CHIP_ID !=(chipid & 0xFC)){
		dev_err(&client->dev, "%s chip id error 0x%x\n",__func__, chipid);
		es9218_power_enable(edata,0);
		goto chip_id_error;
	}
	Chipid = chipid & 0xFC;
	pr_err("%s:chip id: 0x%x\n", __func__, Chipid);
	es9218_chip_init(edata);
    /* es9218 device name */
    i2c_set_clientdata(client, edata);
    if (client->dev.of_node) {
        dev_set_name(&client->dev, "%s", "hifi-es9218");
    } else {
        dev_err(&client->dev, "%s failed to set device name: %d\n", __func__, ret);
    }
    dai = devm_kzalloc(&client->dev, sizeof(es9218_dai), GFP_KERNEL);
    if (!dai) {
	  ret = -1;
        goto err_dai_kzalloc;
    }
    memcpy(dai, es9218_dai, sizeof(es9218_dai));
    pr_info("%s dai->name(%s)\n", __func__, dai->name);

    ret = snd_soc_register_component(&client->dev, &soc_codec_dev_es9218,dai, ARRAY_SIZE(es9218_dai));
    if (ret < 0) {
        dev_err(&client->dev, "%s failed to register hifi es9218: %d\n", __func__, ret);
	 ret = -1;
        goto err_register_codec;
    }else
    	{
    	   pr_info("%s register hifi es9218 ok \r\n",__func__);
    	}
	mutex_init(&edata->lock);

	es9218_write_byte(edata,ES9218_CONTROL_REG14, 0x6a);

	msleep(10);

	if (!IS_ERR_OR_NULL(edata->hifi_dac_regulator)) {
		clk_disable_unprepare(edata->hifi_mclk);
		pr_info("%s: MCLK disable.\n", __func__);
	}
	//es9218_power_enable(edata, 0);  

#ifdef CONFIG_DEBUG_FS
	if (!IS_ERR_OR_NULL(es9218_debugfs_root)) {
		es9218_debugfs = debugfs_create_file("reg",
				0644, es9218_debugfs_root,
				edata, &es9218_debugfs_fops);
		if (!es9218_debugfs) {
			dev_err(&client->dev,"es9218 debugfs reg create fail.\n");
		}

		es9218_debugfs = debugfs_create_file("i2c",
				0644, es9218_debugfs_root,
				edata, &es9218_i2c_debugfs_fops);
		if (!es9218_debugfs) {
			dev_err(&client->dev,"es9218 debugfs i2c create fail.\n");
		}
		es9218_debugfs = debugfs_create_file("chipid",
				0644, es9218_debugfs_root,
				edata, &es9218_chipid_debugfs_fops);
		if (!es9218_debugfs) {
			dev_err(&client->dev,"es9218 debugfs i2c create fail.\n");
		}
	}
#endif

#ifdef CONFIG_HISENSE_PRODUCT_DEVINFO
	hifi_register_productinfo();
#endif /* CONFIG_HISENSE_PRODUCT_DEVINFO */

	/* For debug by --. */
	if (!IS_ERR_OR_NULL(edata->hifi_dac_regulator)) {
		ret = regulator_enable(edata->hifi_dac_regulator);
		pr_info("%s: power up hifi dac.\n", __func__);
	}

      pr_info("%s: exit.\n", __func__);
	return ret;

err_register_codec:

err_dai_kzalloc:
	devm_kfree(&client->dev, dai);
	dai = NULL;
chip_id_error:
err_osci_gpio_request:
      devm_gpio_free(&client->dev,edata->osci_gpio);
	  devm_gpio_free(&client->dev,edata->dac_pwr_gpio);
	  devm_gpio_free(&client->dev,edata->pa_pwr_gpio);
err_sel_gpio_request:
	  devm_gpio_free(&client->dev,edata->hifi_sel_gpio);
	  devm_gpio_free(&client->dev,edata->hph_sel_gpio);
      devm_gpio_free(&client->dev,edata->rst_gpio);	  
err_regmap:
err_es9218_data:
	kfree(edata);

    return ret;
}

static int es9218_i2c_remove(struct i2c_client *client)
{
	pr_info("%s: entry.\n", __func__);
     snd_soc_unregister_component(&client->dev);

    if (gpio_is_valid(pes9218_data->hifi_sel_gpio))
        devm_gpio_free(&client->dev, pes9218_data->hifi_sel_gpio);
	if (gpio_is_valid(pes9218_data->hph_sel_gpio))
        devm_gpio_free(&client->dev, pes9218_data->hph_sel_gpio);
    if (gpio_is_valid(pes9218_data->rst_gpio))
        devm_gpio_free(&client->dev, pes9218_data->rst_gpio);  
    /*ZSQ add oscillator gpio control */
    if (gpio_is_valid(pes9218_data->osci_gpio))
		devm_gpio_free(&client->dev, pes9218_data->osci_gpio);
	if (gpio_is_valid(pes9218_data->dac_pwr_gpio))
		devm_gpio_free(&client->dev, pes9218_data->dac_pwr_gpio);
	if (gpio_is_valid(pes9218_data->pa_pwr_gpio))
		devm_gpio_free(&client->dev, pes9218_data->pa_pwr_gpio);
    pes9218_data=NULL;
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id device_es9218_of_match[] = {
    {.compatible = "ess,hifi-es9218",},
    {},
};
#else
#define device_es9218_of_match 0
#endif

static const struct i2c_device_id es9218_i2c_id[] = {
	{ ES9218_DEV_NAME, 0 },
};

static struct i2c_driver es9218_i2c_driver = {
    .probe		= es9218_i2c_probe,
    .remove		= es9218_i2c_remove,
    .driver = {
		.name	= ES9218_DEV_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = device_es9218_of_match,
    },
    .id_table = es9218_i2c_id,
};

static int __init es9218_dac_init(void)
{
	/* for debugfs root */
#ifdef CONFIG_DEBUG_FS
	es9218_debugfs_root = debugfs_create_dir("audio-es9218", NULL);
	if (!es9218_debugfs_root) {
		pr_err("%s debugfs create dir error\n", __func__);
	} else if (IS_ERR(es9218_debugfs_root)) {
		pr_err("%s Kernel not support debugfs \n", __func__);
		es9218_debugfs_root = NULL;
	}
#endif

	/* add i2c driver */
	if (i2c_add_driver(&es9218_i2c_driver)) {
		pr_err("%s: add i2c driver error\n ", __func__);
	}
    return 0;
}

module_init(es9218_dac_init);
static void __exit es9218_i2c_exit(void)
{
    i2c_del_driver(&es9218_i2c_driver);
}
module_exit(es9218_i2c_exit);


MODULE_DESCRIPTION("es9218 driver");
MODULE_AUTHOR("-- <--@Hisense.com>");
MODULE_LICENSE("GPL");
