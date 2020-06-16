#include "pvd.h"

void PVD_Init(void)
{
	__HAL_RCC_PWR_CLK_ENABLE();
	
	PWR_PVDTypeDef PWR_PVDStruct;
	
	PWR_PVDStruct.PVDLevel = PWR_PVDLEVEL_7;			 // 2.9V
	PWR_PVDStruct.Mode = PWR_PVD_MODE_IT_RISING; 	//������ֵ��ѹʱ����

	HAL_PWR_ConfigPVD(&PWR_PVDStruct);
	
	HAL_PWR_EnablePVD();
}

/* pvd �ص�����--�ڵ���˲��ر�����������*/	
void HAL_PWR_PVDCallback(void)
{
		AllCoils_Stop();
}
