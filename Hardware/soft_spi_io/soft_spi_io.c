#include "soft_spi_io.h" 

void soft_spi_Init(void){
	soft_spi_io_Init();
	cs = 1;
	sclk = 0;
	delay_ms(sclk_period_ms);
};

void soft_spi_io_Init(void){    	 
  GPIO_InitTypeDef  GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);//ʹ��GPIOCʱ��

  //GPIOC6,C7,C8��ʼ������
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//��ͨ���ģʽ
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
  GPIO_Init(GPIOC, &GPIO_InitStructure);//��ʼ��GPIO
	
	
	//PC9--MISO
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//��ͨ����ģʽ
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100M
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;//����
  GPIO_Init(GPIOC, &GPIO_InitStructure);
	
}
uint32_t soft_spi_r_and_s(uint32_t send)
{
	uint32_t read = 0;
	int i = 31;
	for(i = 31; i >= 0; i--)
	{
		cs = 0;
		mosi = (send & 1 << i) >> i;
		sclk = 1;
		delay_ms(sclk_period_ms);
		sclk = 0;
		read += miso << i;
		//printf("%d", miso);
		delay_ms(sclk_period_ms);
	}
	cs = 1;
	printf("\r\n%d\r\n", read);
	return read;
}





