#include "IMU_spi.h"
#ifdef __cplusplus
extern "C" {
#endif
static uint8_t mode = 0;
static uint8_t bits = 8;
static uint8_t count = 2;
static uint32_t speed = 10000000;
static uint16_t delay = 0;

static void pabort(const char *s) {
	perror(s);
	abort();
}
int SPIInit(int spi_num,int cs_num){
    int ret = 0;
    int fd;
    char spi_dev[30];
    sprintf(spi_dev,"/dev/spidev%d.%d",spi_num,cs_num);
    

    fd = open(spi_dev, O_RDWR);
	if (fd < 0) {
		pabort("can't open device");
	}

	// SPI mode
	ret = ioctl(fd, SPI_IOC_WR_MODE, &mode);
	if (ret == -1) {
		pabort("can't set spi mode");
	}

	ret = ioctl(fd, SPI_IOC_RD_MODE, &mode);
	if (ret == -1) {
		pabort("can't get spi mode");
	}


	// bits per word
	ret = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
	if (ret == -1) {
		pabort("can't set bits per word");
	}

	ret = ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bits);
	if (ret == -1) {
		pabort("can't get bits per word");
	}


	// max speed Hz
	ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
	if (ret == -1) {
		pabort("can't set max speed hz");
	}

	ret = ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed);
	if (ret == -1) {
		pabort("can't get max speed hz");
	}

	printf("spi mode: %d\n", mode);
	printf("bits per word: %d\n", bits);
	printf("max speed: %d Hz (%d KHz)\n", speed, speed/1000);

    return fd;
}
void SPIClose(int *fd_address){
    close(*fd_address);
}
uint8_t SPI_ReadOneByte(int fd, uint8_t reg){
    int8_t *tx_buffer;
    int8_t rx_buffer[2]={0,0};
	uint8_t readBuf;
    int ret;
    
	tx_buffer = (char *)calloc(count, sizeof(char));
	if (tx_buffer == 0) {
		pabort("failed to malloc tx_buffer");
	}

	tx_buffer[0] = 0x80 | reg; // setting msb to 1 makes this a "read" operation

	struct spi_ioc_transfer tr = {
		.tx_buf = (unsigned long)tx_buffer,
		.rx_buf = (unsigned long)&rx_buffer,
		.len = count,
		.delay_usecs = delay,
		.speed_hz = speed,
		.bits_per_word = bits,
	};

	ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
	if (ret < 1) {
		pabort("can't send spi message");
	}

	free(tx_buffer);
    readBuf = rx_buffer[1];

	return readBuf;
}
void SPI_WriteOneByte(int fd, uint8_t reg, uint8_t value){
    int8_t *tx_buffer;
    int8_t rx_buffer[2]={0,0};
	int ret;
	
	tx_buffer = (char *)calloc(count, sizeof(char));
	if (tx_buffer == 0) {
		pabort("failed to malloc tx_buffer");
	}

	tx_buffer[0] = reg; // setting msb to 0 makes this a "write" operation
	tx_buffer[1] = value;
	struct spi_ioc_transfer tr = {
		.tx_buf = (unsigned long)tx_buffer,
		.rx_buf = (unsigned long)&rx_buffer,
		.len = count,
		.delay_usecs = delay,
		.speed_hz = speed,
		.bits_per_word = bits,
	};
	ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
	if (ret < 1) {
		pabort("can't write spi message");
	}

	free(tx_buffer);
	return;
}



float invSqrt(float x){
  float halfx = 0.5f * x;
  float y = x;
  
  long i = *(long*)&y;                //get bits for floating value
  i = 0x5f3759df - (i >> 1);          //gives initial guss you
  y = *(float*)&i;                    //convert bits back to float
  y = y * (1.5f - (halfx * y * y));   //newtop step, repeating increases accuracy
  
  return y;
}
#ifdef __cplusplus
}
#endif
