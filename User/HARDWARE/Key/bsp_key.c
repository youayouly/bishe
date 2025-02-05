#include "./key/bsp_key.h"  

/**************************************************************************
Function: Key_GPIO_Config
Input   : none
Output  : none
�������ܣ���������
��ڲ���: �� 
����  ֵ����
**************************************************************************/	 	
void Key_GPIO_Config(void)
{
	
	GPIO_InitTypeDef GPIO_InitStructure;
	
	/*���������˿ڵ�ʱ��*/
	RCC_APB2PeriphClockCmd(KEY1_GPIO_CLK|KEY2_GPIO_CLK,ENABLE);
	
	//ѡ�񰴼�������
	GPIO_InitStructure.GPIO_Pin = KEY1_GPIO_PIN; 
	// ���ð���������Ϊ��������
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; 
	//ʹ�ýṹ���ʼ������
	GPIO_Init(KEY1_GPIO_PORT, &GPIO_InitStructure);
	
	//ѡ�񰴼�������
	GPIO_InitStructure.GPIO_Pin = KEY2_GPIO_PIN; 
	//���ð���������Ϊ��������
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; 
	//ʹ�ýṹ���ʼ������
	GPIO_Init(KEY2_GPIO_PORT, &GPIO_InitStructure);	
	
}

/**************************************************************************
Function: Key_Scan
Input   : none
Output  : none
�������ܣ��������
��ڲ���: �� 
����  ֵ����
**************************************************************************/	 
//����Ƿ񵥻�����
uint8_t Key_Scan(GPIO_TypeDef* GPIOx,uint16_t GPIO_Pin)
{			
	/*����Ƿ��а������� */
	if(GPIO_ReadInputDataBit(GPIOx,GPIO_Pin) == KEY_ON )  
	{	 
		/*�ȴ������ͷ� */
		while(GPIO_ReadInputDataBit(GPIOx,GPIO_Pin) == KEY_ON);   
		return 	KEY_ON;	 
	}
	else
		return KEY_OFF;
}

/**************************************************************************
Function: User_Key_Scan
Input   : none
Output  : Key status
�������ܣ��û�������⣨����,������˫����
��ڲ���: �� 
����  ֵ�����ذ���״̬
**************************************************************************/	 
//����5ms�ж��е���
uint8_t User_Key_Scan(void)
{
	static u16 count_time = 0;					//���㰴�µ�ʱ�䣬ÿ5ms��1
	static u8 key_step = 0;						//��¼��ʱ�Ĳ���
	switch(key_step)
	{
		case 0:
			if(GPIO_ReadInputDataBit(KEY1_GPIO_PORT,KEY1_GPIO_PIN) == KEY_ON )
				key_step++;						//��⵽�а������£�������һ��
			break;
		case 1:
			if((++count_time) == 5)				//��ʱ����
			{
				if(GPIO_ReadInputDataBit(KEY1_GPIO_PORT,KEY1_GPIO_PIN) == KEY_ON )//����ȷʵ������
					key_step++,count_time = 0;	//������һ��
				else
					count_time = 0,key_step = 0;//����λ
			}
			break;
		case 2:
			if(GPIO_ReadInputDataBit(KEY1_GPIO_PORT,KEY1_GPIO_PIN) == KEY_ON )
				count_time++;					//���㰴�µ�ʱ��
			else 								//��ʱ���ɿ���
				key_step++;						//������һ��
			break;
		case 3:									//��ʱ�����µ�ʱ�䣬���ж��ǳ������Ƕ̰�
			if(count_time > 400)				//��5ms�ж��е��ã��ʰ���ʱ��������400*5 = 2000ms�����ֵ��
			{							
				key_step = 0;					//��־λ��λ
				count_time = 0;
				return Long_Press;				//���� ���� ��״̬ 
 			}
			else if(count_time > 5)				//��ʱ�ǵ�����һ��
			{
				key_step++;						//��ʱ������һ�����ж��Ƿ���˫��
				count_time = 0;					//���µ�ʱ������
			}
			else
			{
				key_step = 0;
				count_time = 0;	
			}
			break;
		case 4:									//�ж��Ƿ���˫���򵥻�
			if(++count_time == 38)				//5*38 = 190ms���жϰ����Ƿ���
			{
				if(GPIO_ReadInputDataBit(KEY1_GPIO_PORT,KEY1_GPIO_PIN) == KEY_ON )	//����ȷʵ������
				{																	//����˫�����ܰ�̫�죬��ʶ��ɵ���
					key_step++;														//������һ������Ҫ�����ֲ����ͷ�״̬
					count_time = 0;
				}
				else																//190ms���ް������£���ʱ�ǵ�����״̬
				{
					key_step = 0;				//��־λ��λ
					count_time = 0;					
					return Click;				//���ص�����״̬
				}
			}
			break;
		case 5:
			if(GPIO_ReadInputDataBit(KEY1_GPIO_PORT,KEY1_GPIO_PIN) == KEY_ON )//�������ڰ���
			{
				count_time++;
			}
			else								//�����Ѿ�����
			{
//				if(count_time>400)				//����ڶ��εĵ���Ҳ�����ж�ʱ��ģ�Ĭ�ϲ��ж�ʱ�䣬ȫ��������˫��
//				{
//				}
				count_time = 0;
				key_step = 0;
				return Double_Click;
			}
			break;
		default:break;
	}
	return No_Action;							//�޶���

}

/**************************************************************************
Function: User_Key_Val
Input   : none
Output  : Key status
�������ܣ�����ֵ���
��ڲ���: �� 
����  ֵ����
**************************************************************************/	 
//����5ms�ж��е���
void Get_KeyVal(void)
{
	switch(User_Key_Scan())								//����ɨ�裬������˫���򳤰�
		{
			//�������������л���
			//1.��ͨң��ģʽ
			//2.�״�Ѳ��ģʽ
			//3.�״����ģʽ
			//4.���Ѳ��ģʽ
			//5.CCDѲ��ģʽ
			//��������������λ��
			case Click:
				Mode++;
				if(Mode == ELE_Line_Patrol_Mode)
					ELE_ADC_Init();							//��ʼ�����Ѳ��ģʽ
				
				else if(Mode == CCD_Line_Patrol_Mode)		//CCDѲ��ģʽ
				{
					CCD_Init();								//CCD��ʼ����CCDģ��͵��Ѳ��ģ�鹲��һ���ӿڣ���������ͬʱʹ��
					BEEP_OFF;								//�رշ�����
				}
				
				else if(Mode == 6)							//6��ģʽѭ���л�
				{
					Mode = Normal_Mode;
					Move_X = Move_Z = 0;
					MotorA.Motor_Pwm = MotorB.Motor_Pwm = 0;
					BEEP_OFF;
				}
				break;
			case Long_Press:
				Flag_Show = !Flag_Show;								//���� ����/�˳� ��λ��ģʽ
				break;
			case Double_Click:										
				if(Mode == ELE_Line_Patrol_Mode)					//���Ѳ��״̬ʱ��˫�����Դ�/�ر��״����ϰ��Ĭ�ϴ�
				{
					Lidar_Detect = !Lidar_Detect;
					if(Lidar_Detect == Lidar_Detect_OFF)
						memset(Dataprocess,0, sizeof(PointDataProcessDef)*390);		//�����״����ϰ������������
				}
				break;
			default:break;
		}
}

/************************************END OF FILE*******************************/
