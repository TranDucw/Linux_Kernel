#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <i2c/smbus.h>
#include <math.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <string.h>

#define I2C_addr         0x68

#define Acc_X_Addr       59
#define Acc_Y_Addr       61

#define Gyro_X_Addr      67
#define Gyro_Y_Addr      69


int led;
int max_speed = 10000000;
int bpw = 0;
int mpu;

/*mpu program*/ 

//ghi du lieu xuong cac thanh ghi
void write_Data(int reg_addr, int data){
    int ret = i2c_smbus_write_word_data(mpu, reg_addr, data);
    if(ret < 0){
        printf("Can't send data 1\n");
        exit(1);
    }
    ret = i2c_smbus_read_word_data(mpu, reg_addr);
}

//khoi tao che do hoat dong cua mpu
void Mpu_Init(){
    write_Data(25, 2);
    write_Data(26, 2);
    write_Data(27, 8);
    write_Data(28, 0);
    write_Data(107, 1);
}

//doc du lieu gia toc tra ve
float get_Acc(int addr){
    int16_t a = 0;
    int16_t b = 0;
    a = i2c_smbus_read_word_data(mpu, addr);
    b = i2c_smbus_read_word_data(mpu, addr+1);
    a = (a<<8)|b;
    float acc = a/16384.0;
    return acc;
}

//doc du lieu gyro
float get_Gyro(int addr){
    int16_t a = 0;
    int16_t b = 0;
    a = i2c_smbus_read_word_data(mpu, addr);
    b = i2c_smbus_read_word_data(mpu, addr+1);
    float gyro = a/65.5;
    return gyro;
}

//tinh goc thong qua gia toc thu duoc
float get_Angle (int addr1, int addr2){
    float angle = atan(get_Acc(addr1)/get_Acc(addr2))*180/3.14;
    return angle;
}

/*led program*/ 

// tinh gia tri tuyet doi
float Abs(float angle){
    if (angle < 0)
        return angle*(-1);
    return angle;
}

//viet ham truyen du lieu cho cac ngoai vi spi
void sendata(uint8_t address, uint8_t value){
    struct spi_ioc_transfer spi;
    memset(&spi, 0, sizeof(spi));
    uint8_t data[2];
    data[0]             = address;
    data[1]             = value;
    spi.tx_buf          = (unsigned long) data;
    spi.len             = 2;
    spi.delay_usecs     = 0;
    spi.speed_hz        = max_speed;
    spi.bits_per_word   = bpw;
    ioctl (led, SPI_IOC_MESSAGE(1), &spi);
}

//khoi tao che do hoat dong cua led
void Led_Init(){
    sendata(0x09, 0x00); // decode mode
    sendata(0x0a, 10); // intensity
    sendata(0x0b, 7); // display
    sendata(0x0c, 1); // on/off
    sendata(0x0f, 0); // operate mode
}

//tat den led
void Off_Led(){
    for (int i=1; i<9; i++){
        sendata(i, 0x00);
    }
}

//hien thi led theo du lieu truyen vao
void Display (uint8_t Led_Code[]){
    for (int i=1; i<9; i++){
        sendata(i,Led_Code[i-1]);
    }
}

//test led
void Test_Led(){
    Off_Led();
    for (int i=1; i<9; i++){
        sendata(i, 0xff);
        usleep(125000);
        Off_Led();
    }
}

//che do canh bao
void Warning(float angle){
    uint8_t Led_Code_1[8] = {0x08, 0x18, 0x08, 0x08, 0x08, 0x08, 0x08, 0x1C}; // ma so mot
    uint8_t Led_Code_2[8] = {0x3C, 0x66, 0x66, 0x0C, 0x18, 0x30, 0x7E, 0x7E}; // ma so hai
    uint8_t Led_Code_3[8] = {0x7E, 0x0C, 0x18, 0x3C, 0x06, 0x06, 0x46, 0x3C}; // ma so ba
    Off_Led();
    if (Abs(angle)<15){
        Display(Led_Code_1);
    }
    else if (Abs(angle)<30){
        Display(Led_Code_2);
    }
    else if (Abs(angle)<45){
        Display(Led_Code_3);
    }
    else {
        Display(Led_Code_3);
        for (int i = 0; i<16; i++){
            sendata(0x0a, i);
            usleep(100000);
        }
        for (int i = 0; i<16; i++){
            sendata(0x0a, 15-i);
            usleep(100000);
        }
    }
}

/*main program*/

int main(void){
    float acc_X, acc_Y;
    float gyro_X, gyro_Y;
    /*Load I2C Driver*/
    mpu = open("/dev/i2c-1", O_RDWR);
    if(mpu < 0){
        printf("Can't load I2C driver \n");
        exit(1);
    }
    // set slave address
    if(ioctl(mpu, I2C_SLAVE, I2C_addr) < 0){
        printf("Can't set I2C address \n");
        exit(1);
    }
    /* */
    /*Load SPI driver*/
    led = open ("/dev/spidev0.0",O_RDWR);
    if (led<0){
        printf ("loi\n");
        exit(1);
    }
    int mode = 0;
    if(ioctl(led, SPI_IOC_WR_MODE, &mode)<0){
        printf("khong set mode duoc");
        exit(1);
    }
    int lsb = 0;
    if(ioctl(led, SPI_IOC_WR_LSB_FIRST, &lsb)<0){
        printf("khong set msb duoc");
        exit(1);
    }
    if(ioctl(led, SPI_IOC_WR_BITS_PER_WORD, &bpw)<0){
        printf("khong set msb duoc");
        exit(1);
    }
    if(ioctl(led, SPI_IOC_WR_MAX_SPEED_HZ, &max_speed)<0){
        printf("speed");
        exit(1);
    }
    /**/

    Mpu_Init();
    Led_Init();

    Test_Led();  

    while(1){
        float angle = get_Angle(Acc_X_Addr, Acc_Y_Addr);
        if (angle<45)
            sendata(0x0a, 10);
        printf ("%f\n", angle);
        Warmning(angle);
        sleep(1);
    }
    return 0;
}