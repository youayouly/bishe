/***********************************************
��˾����Ȥ�Ƽ�����ݸ�����޹�˾
Ʒ�ƣ�WHEELTEC
������wheeltec.net
�Ա����̣�shop114407458.taobao.com 
����ͨ: https://minibalance.aliexpress.com/store/4455017
�汾��V1.0
�޸�ʱ�䣺2023-03-02

Brand: WHEELTEC
Website: wheeltec.net
Taobao shop: shop114407458.taobao.com 
Aliexpress: https://minibalance.aliexpress.com/store/4455017
Version: V1.0
Update��2023-03-02

All rights reserved
***********************************************/



MiniBalance:
����STARTUP
��  ����startup_stm32f10x_hd.s			//stm32�����ļ�
��
����CMSIS								//Cortex΢�������ӿ�
��  ����core_cm3.c
��  ����core_cm3.h
��  ����......
��
����FWIB								//stm32��׼���ļ�
��  ����misc.c
��  ����stm32f10x_adc.c
��  ����stm32f10x_bkp.c
��  ����......
��
����USER								//��Ҫ���û��ļ���main�������
��  ����main.c							//main�������
��  ����stm32f10x_conf.h					//�����ļ���
��  ����stm32f10x_it.c
��  ����stm32f10x_it.h
��  ����Header.h							//����ͷ�ļ�
��  
����HARDWARE							//��Ҫ��Ӳ�����ֵĴ���
��  ����bsp_beep.c				
��  ����bsp_key.c
��  ����bsp_led.c
��  ����bsp_SysTick.c
��  ����......
��
����CONTROL								//�����߼����ִ���
��  ����control.c						//���ƺ������⣬5ms�ж�
��  ����show.c							//��ʾ��������app��ʾ��������λ��ʾ����
��  ����bluetooth.c						//�������ֿ��ƴ���
��  ����pid.c							//pid����
��  ����Lidar.c							//�״ﴮ�ڽӿںʹ������
��  ����capture.c						//��Ҫ�ǲ����ܣ�δʹ��
��  ����ELE_CCD.c						//���Ѳ�ߺ�CCDѲ��
��
����MPU6050								//MPU6050
��  ����inv_mpu.c						//MPU6050��ؿ��ļ�
��  ����inv_mpu_dmp_motion_driver.c		//MPU6050��ؿ��ļ�
��  ����IOI2C.c							//I2C�����ļ�
��  ����MPU6050.c						//MPU6050�����ļ�
��  ����filter.c							//�˲��ļ����������ͻ����˲�
��  ����MPU6050_Test.c					//MPU6050���Գ���
��
����DOC									//�ĵ�˵��
   ����readme.txt

