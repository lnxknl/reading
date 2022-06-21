// lcd snips
int	lcd5110_open(struct inode *inode, struct file *filp);
int	lcd5110_release(struct inode *inode, struct file *filp);
ssize_t	lcd5110_read(struct file *filp, char *buf,
                       size_t count, loff_t *f_pos);
ssize_t lcd5110_write(struct file *filp, char *buf,
                       size_t count, loff_t *f_pos);
void	lcd5110_exit(void);
int		lcd5110_init(void);


/* file access fcuntions */
struct file_operations lcd5110_fops = {
	read:		lcd5110_read,
	write:		lcd5110_write,
	open:		lcd5110_open,
	release:	lcd5110_release
};

int lcd5110_major = 61;

#define BCM2708_PERI_BASE        0x20000000
#define GPIO_BASE                (BCM2708_PERI_BASE + 0x200000) /* GPIO controller */

#define PAGE_SIZE (4*1024)
#define BLOCK_SIZE (4*1024)

#define RES   17 // 0
#define SCE   18 // 1
#define DC    27 // 2
#define SDIN  22 // 3
#define SCLK  23 // 4
#define TMP0  24 // 5
#define TMP1  25 // 6

#define LCD_WIDTH 84
#define LCD_HEIGHT 48

#define LCD_C	0
#define LCD_D	1

void initLcdScreen(void);
void clearLcdScreen(void);
void sendByteToLcd(bool, unsigned char);
void writeCharToLcd(char);
void writeStringToLcd(char *);

		gpio_free(pins[g]);
	char *kbuf = kcalloc((count + 1), sizeof(char), GFP_KERNEL);

ssize_t lcd5110_write( struct file *filp, char *ubuf,
	const size_t count, loff_t *f_pos)
{
	/* Buffer writing to the device */
	char *kbuf = kcalloc((count + 1), sizeof(char), GFP_KERNEL);

	if(copy_from_user(kbuf, ubuf, count) != 0)
	{
		kfree(kbuf);
		return -EFAULT;
	}

	kbuf[count-1] = 0;

	clearLcdScreen();
	writeStringToLcd(kbuf);
	kfree(kbuf);

	return count;
}

void initLcdScreen()
{
	// set GPIOs
	gpio_set_value(SCE, false);
	gpio_set_value(SCLK, false);
	gpio_set_value(RES, false);
	udelay(2);
	gpio_set_value(RES, true);

	// init LCD
	sendByteToLcd(LCD_C, 0x21);	// LCD Extended Commands
	sendByteToLcd(LCD_C, 0xb1);	// Set LCD Cop (Contrast).	//0xb1
	sendByteToLcd(LCD_C, 0x04);	// Set Temp coefficent.		//0x04
	sendByteToLcd(LCD_C, 0x14);	// LCD bias mode 1:48. 		//0x13
	sendByteToLcd(LCD_C, 0x0c);	// LCD in normal mode. 0x0d inverse mode
	sendByteToLcd(LCD_C, 0x20);
	sendByteToLcd(LCD_C, 0x0c);

	clearLcdScreen();
}

void sendByteToLcd(bool cd, unsigned char data)
{
	if(cd)
		gpio_set_value(DC, true);
	else
		gpio_set_value(DC, false);

	unsigned char pattern = 0b10000000;
	for(int i=0; i < 8; i++)
	{
		gpio_set_value(SCLK, false);
		if(data & pattern)
			gpio_set_value(SDIN, true);
		else
			gpio_set_value(SDIN, false);

		udelay(1);
		gpio_set_value(SCLK, true);
		udelay(1);
		pattern >>= 1;
	}
}


void clearLcdScreen()
{
	for(int i=0; i < LCD_WIDTH * LCD_HEIGHT / 8; i++)
		sendByteToLcd(LCD_D, 0x00);

	sendByteToLcd(LCD_C, 0x80 | 0); // set x coordinate to 0
	sendByteToLcd(LCD_C, 0x40 | 0); // set y coordinate to 0
}


void writeCharToLcd(char data)
{
	sendByteToLcd(LCD_D, 0x00);
	for(int i=0; i < 5; i++)
		sendByteToLcd(LCD_D, ASCII[data-0x20][i]);
	sendByteToLcd(LCD_D, 0x00);
}
void writeStringToLcd(char *data)
{
	while(*data)
		writeCharToLcd(*data++);
}



//=====

void lcd_send(uint8_t data, uint8_t rs)
{
	// Write logic one to send characters or logic 0 to send a command
	if (rs)
		lcd_iohigh(E_RS_PIN);
	else
		lcd_iolow(E_RS_PIN);
	// Clear the RW pin if used
	lcd_iolow(E_RW_PIN);

	if (iomode == 4) {
		lcd_iowrite4(data >> 4);
		lcd_iowrite4(data);
	} else {
		lcd_iowrite8(data);
	}
}



//====

int ili9341_write_spi(struct ili9341 *item, void *buf, size_t len)
{
	struct spi_transfer t = {
		.tx_buf = buf,
		.len = len,
	};
	struct spi_message m;

	if (!item->spi) {
		dev_err(item->info->device,
			"%s: par->spi is unexpectedly NULL\n", __func__);
		return -1;
	}

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	return spi_sync(item->spi, &m);
}


static int ili9341_init_gpio(struct ili9341 *item)
{
	//DC high - data, DC low - command
	return gpio_request_one(ILI_GPIO_DC, GPIOF_OUT_INIT_HIGH,
		item->info->device->driver->name);
}


static void ili9341_write_data(struct ili9341 *item, unsigned char dc, unsigned char value) {

	if (dc == ILI_COMMAND) {
		gpio_set_value(ILI_GPIO_DC, 0);
		ili9341_write_spi_1_byte(item, value);
		gpio_set_value(ILI_GPIO_DC, 1);
	} else { //ILI_DATA
		ili9341_write_spi_1_byte(item, value);
	}

}


/* Init sequence taken from: Arduino Library for the Adafruit 2.2" display */
static int ili9341_init_display(struct ili9341 *item)
{
	/* Software Reset */
	ili9341_write_data(item, ILI_COMMAND, 0x01);

	mdelay(120);

	/* Display OFF */
	ili9341_write_data(item, ILI_COMMAND, 0x28);

	ili9341_write_data(item, ILI_COMMAND, 0xEF);
	ili9341_write_data(item, ILI_DATA, 0x03);
	ili9341_write_data(item, ILI_DATA, 0x80);
	ili9341_write_data(item, ILI_DATA, 0x02);

	ili9341_write_data(item, ILI_COMMAND, 0xCF);
	ili9341_write_data(item, ILI_DATA, 0x00);
	ili9341_write_data(item, ILI_DATA, 0xC1);
	ili9341_write_data(item, ILI_DATA, 0x30);

	ili9341_write_data(item, ILI_COMMAND, 0xED);
	ili9341_write_data(item, ILI_DATA, 0x64);
	ili9341_write_data(item, ILI_DATA, 0x03);
	ili9341_write_data(item, ILI_DATA, 0x12);
	ili9341_write_data(item, ILI_DATA, 0x81);

	ili9341_write_data(item, ILI_COMMAND, 0xE8);
	ili9341_write_data(item, ILI_DATA, 0x85);
	ili9341_write_data(item, ILI_DATA, 0x00);
	ili9341_write_data(item, ILI_DATA, 0x78);

	ili9341_write_data(item, ILI_COMMAND, 0xCB);
	ili9341_write_data(item, ILI_DATA, 0x39);
	ili9341_write_data(item, ILI_DATA, 0x2C);
	ili9341_write_data(item, ILI_DATA, 0x00);
	ili9341_write_data(item, ILI_DATA, 0x34);
	ili9341_write_data(item, ILI_DATA, 0x02);

	ili9341_write_data(item, ILI_COMMAND, 0xF7);
	ili9341_write_data(item, ILI_DATA, 0x20);

	ili9341_write_data(item, ILI_COMMAND, 0xEA);
	ili9341_write_data(item, ILI_DATA, 0x00);
	ili9341_write_data(item, ILI_DATA, 0x00);

	/* Power Control 1 */
	ili9341_write_data(item, ILI_COMMAND, 0xC0);
	ili9341_write_data(item, ILI_DATA, 0x23);

	/* Power Control 2 */
	ili9341_write_data(item, ILI_COMMAND, 0xC1);
	ili9341_write_data(item, ILI_DATA, 0x10);

	/* VCOM Control 1 */
	ili9341_write_data(item, ILI_COMMAND, 0xC5);
	ili9341_write_data(item, ILI_DATA, 0x3e);
	ili9341_write_data(item, ILI_DATA, 0x28);

	/* VCOM Control 2 */
	ili9341_write_data(item, ILI_COMMAND, 0xC7);
	ili9341_write_data(item, ILI_DATA, 0x86);

	/* COLMOD: Pixel Format Set */
	/* 16 bits/pixel */
	ili9341_write_data(item, ILI_COMMAND, 0x3A);
	ili9341_write_data(item, ILI_DATA, 0x55);

	/* Frame Rate Control */
	/* Division ratio = fosc, Frame Rate = 79Hz */
	ili9341_write_data(item, ILI_COMMAND, 0xB1);
	ili9341_write_data(item, ILI_DATA, 0x00);
	ili9341_write_data(item, ILI_DATA, 0x18);

	/* Display Function Control */
	ili9341_write_data(item, ILI_COMMAND, 0xB6);
	ili9341_write_data(item, ILI_DATA, 0x08);
	ili9341_write_data(item, ILI_DATA, 0x82);
	ili9341_write_data(item, ILI_DATA, 0x27);

	/* MADCTL, required to resolve 'mirroring' effect */
	ili9341_write_data(item, ILI_COMMAND, 0x36);
	if (mode_BGR)	{
		ili9341_write_data(item, ILI_DATA, 0x48);
		ili9341_set_display_options(item);
		printk("COLOR LCD in BGR mode\n");
	} else 	{
		ili9341_write_data(item, ILI_DATA, 0x40);
		ili9341_set_display_options(item);
		printk("COLOR LCD in RGB mode\n");
	}


	/* Gamma Function Disable */
	ili9341_write_data(item, ILI_COMMAND, 0xF2);
	ili9341_write_data(item, ILI_DATA, 0x00);

	/* Gamma curve selected  */
	ili9341_write_data(item, ILI_COMMAND, 0x26);
	ili9341_write_data(item, ILI_DATA, 0x01);

	/* Positive Gamma Correction */
	ili9341_write_data(item, ILI_COMMAND, 0xE0);
	ili9341_write_data(item, ILI_DATA, 0x0F);
	ili9341_write_data(item, ILI_DATA, 0x31);
	ili9341_write_data(item, ILI_DATA, 0x2B);
	ili9341_write_data(item, ILI_DATA, 0x0C);
	ili9341_write_data(item, ILI_DATA, 0x0E);
	ili9341_write_data(item, ILI_DATA, 0x08);
	ili9341_write_data(item, ILI_DATA, 0x4E);
	ili9341_write_data(item, ILI_DATA, 0xF1);
	ili9341_write_data(item, ILI_DATA, 0x37);
	ili9341_write_data(item, ILI_DATA, 0x07);
	ili9341_write_data(item, ILI_DATA, 0x10);
	ili9341_write_data(item, ILI_DATA, 0x03);
	ili9341_write_data(item, ILI_DATA, 0x0E);
	ili9341_write_data(item, ILI_DATA, 0x09);
	ili9341_write_data(item, ILI_DATA, 0x00);

	/* Negative Gamma Correction */
	ili9341_write_data(item, ILI_COMMAND, 0xE1);
	ili9341_write_data(item, ILI_DATA, 0x00);
	ili9341_write_data(item, ILI_DATA, 0x0E);
	ili9341_write_data(item, ILI_DATA, 0x14);
	ili9341_write_data(item, ILI_DATA, 0x03);
	ili9341_write_data(item, ILI_DATA, 0x11);
	ili9341_write_data(item, ILI_DATA, 0x07);
	ili9341_write_data(item, ILI_DATA, 0x31);
	ili9341_write_data(item, ILI_DATA, 0xC1);
	ili9341_write_data(item, ILI_DATA, 0x48);
	ili9341_write_data(item, ILI_DATA, 0x08);
	ili9341_write_data(item, ILI_DATA, 0x0F);
	ili9341_write_data(item, ILI_DATA, 0x0C);
	ili9341_write_data(item, ILI_DATA, 0x31);
	ili9341_write_data(item, ILI_DATA, 0x36);
	ili9341_write_data(item, ILI_DATA, 0x0F);

	/* Sleep OUT */
	ili9341_write_data(item, ILI_COMMAND, 0x11);

	mdelay(120);

	/* Display ON */
	ili9341_write_data(item, ILI_COMMAND, 0x29);

	ili9341_clear_graph(item);

	printk("COLOR LCD driver initialized\n");


	return 0;
}

static int __init ili9341_video_alloc(struct ili9341 *item)
{
        unsigned int frame_size;

        dev_dbg(item->dev, "%s: item=0x%p\n", __func__, (void *)item);

        frame_size = item->info->fix.line_length * item->info->var.yres;
        dev_dbg(item->dev, "%s: item=0x%p frame_size=%u\n",
                __func__, (void *)item, frame_size);

        item->pages_count = frame_size / PAGE_SIZE;
        if ((item->pages_count * PAGE_SIZE) < frame_size) {
                item->pages_count++;
        }
        dev_dbg(item->dev, "%s: item=0x%p pages_count=%u\n",
                __func__, (void *)item, item->pages_count);

        item->info->fix.smem_len = item->pages_count * PAGE_SIZE;
        item->info->fix.smem_start =
            (unsigned short*) vmalloc(item->info->fix.smem_len);
        if (!item->info->fix.smem_start) {
                dev_err(item->dev, "%s: unable to vmalloc\n", __func__);
                return -ENOMEM;
        }
        memset((void *)item->info->fix.smem_start, 0, item->info->fix.smem_len);

        return 0;
}

static void ili9341_video_free(struct ili9341 *item)
{
        dev_dbg(item->dev, "%s: item=0x%p\n", __func__, (void *)item);

        kfree((void *)item->info->fix.smem_start);
}

static int __init ili9341_pages_alloc(struct ili9341 *item)
{
        unsigned short pixels_per_page;
        unsigned short yoffset_per_page;
        unsigned short xoffset_per_page;
        unsigned short index;
        unsigned short x = 0;
        unsigned short y = 0;
        unsigned short *buffer;
        unsigned int len;

        dev_dbg(item->dev, "%s: item=0x%p\n", __func__, (void *)item);

        item->pages = kmalloc(item->pages_count * sizeof(struct ili9341_page),
                              GFP_KERNEL);
        if (!item->pages) {
                dev_err(item->dev, "%s: unable to kmalloc for ssd1289_page\n",
                        __func__);
                return -ENOMEM;
        }

        pixels_per_page = PAGE_SIZE / (item->info->var.bits_per_pixel / 8);
        yoffset_per_page = pixels_per_page / item->info->var.xres;
        xoffset_per_page = pixels_per_page -
            (yoffset_per_page * item->info->var.xres);
        dev_dbg(item->dev, "%s: item=0x%p pixels_per_page=%hu "
                "yoffset_per_page=%hu xoffset_per_page=%hu\n",
                __func__, (void *)item, pixels_per_page,
                yoffset_per_page, xoffset_per_page);

        buffer = (unsigned short *)item->info->fix.smem_start;
        for (index = 0; index < item->pages_count; index++) {
                len = (item->info->var.xres * item->info->var.yres) -
                    (index * pixels_per_page);
                if (len > pixels_per_page) {
                        len = pixels_per_page;
                }
                dev_dbg(item->dev,
                        "%s: page[%d]: x=%3hu y=%3hu buffer=0x%p len=%3hu\n",
                        __func__, index, x, y, buffer, len);
                item->pages[index].x = x;
                item->pages[index].y = y;
                item->pages[index].buffer = buffer;
                item->pages[index].len = len;

                x += xoffset_per_page;
                if (x >= item->info->var.xres) {
                        y++;
                        x -= item->info->var.xres;
                }
                y += yoffset_per_page;
                buffer += pixels_per_page;
        }

        return 0;
}

static struct fb_fix_screeninfo ili9341_fix __initdata = {
        .id          = "ILI9341",
        .type        = FB_TYPE_PACKED_PIXELS,
        .visual      = FB_VISUAL_TRUECOLOR,
        .accel       = FB_ACCEL_NONE,
        .line_length = 320 * 2,
};

