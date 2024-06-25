
#include "stm32f1xx_hal.h"
#include <stdarg.h>
#include <stdio.h>
#include "nRF24L01.h"
//#include "delay.h"
//#include <ch376.h>


extern SPI_HandleTypeDef hspi2;
extern UART_HandleTypeDef huart1;
extern TIM_HandleTypeDef htim1;
extern void Send_Data_nrf(char *);

uint8_t downloade_buffer[10000]={0},downloade_flag=0,error_nrf;
uint16_t timer_cun=0;
uint16_t downloade_cun=0,send_cun=0;

void usb_Config(void);
char usb_cmd_write(uint8_t len,uint8_t cmd,...);
uint8_t usb_cmd_int(uint8_t len,uint8_t cmd);
char usb_read(uint8_t len,uint8_t *);
char usb_open(void);
char usb_status(void);
char usb_open_fold(char *) ;
void usb_set_name (char *name);
void usb_dir(void);
void send_file_properties(uint8_t *);
void usb_download_file(char *str);
//void send_pack(uint8_t tx_size,uint8_t *buffer);
void send_pack(void);
char usb_delet_file(char *);	
char usb_delet_folder(char *);
uint8_t save_downloade(uint8_t size,uint8_t *read_buffer);




extern void debug_flash(const char *);
extern void debug( char *);
extern void debug_number( char *pointer,uint32_t number);

#define   wr_ch376_low     HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,GPIO_PIN_RESET)
#define   wr_ch376_high    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,GPIO_PIN_SET)

#define   rd_ch376_low     HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_RESET)
#define   rd_ch376_high    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_SET)

#define   pcs_ch376_low     HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_RESET)
#define   pcs_ch376_high    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_SET)

#define   scs_ch376_low     HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_RESET)
#define   scs_ch376_high    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_SET)

#define   res_ch376_low     HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,GPIO_PIN_RESET)
#define   res_ch376_high    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,GPIO_PIN_SET)




#define    CMD01_GET_IC_VER      0x01          
#define    CMD21_SET_BAUDRATE    0x02          
#define    CMD00_ENTER_SLEEP     0x03            
#define    CMD00_RESET_ALL       0x05            
#define    CMD11_CHECK_EXIST     0x06            
#define    CMD20_CHK_SUSPEND     0x0B            
#define    CMD20_SET_SDO_INT     0x0B            
#define    CMD14_GET_FILE_SIZE   0x0C            
#define    CMD50_SET_FILE_SIZE   0x0D            
#define    CMD11_SET_USB_MODE    0x15            
#define    CMD01_GET_STATUS      0x22            
#define    CMD00_UNLOCK_USB      0x23            
#define    CMD01_RD_USB_DATA0    0x27            
#define    CMD01_RD_USB_DATA     0x28            
#define    CMD10_WR_USB_DATA7    0x2B            
#define    CMD10_WR_HOST_DATA    0x2C            
#define    CMD01_WR_REQ_DATA     0x2D            
#define    CMD20_WR_OFS_DATA     0x2E            
#define    CMD10_SET_FILE_NAME   0x2F           
#define    CMD0H_DISK_CONNECT    0x30           
#define    CMD0H_DISK_MOUNT      0x31            
#define    CMD0H_FILE_OPEN       0x32            
#define    CMD0H_FILE_ENUM_GO    0x33            
#define    CMD0H_FILE_CREATE     0x34            
#define    CMD0H_FILE_ERASE      0x35            
#define    CMD1H_FILE_CLOSE      0x36            
#define    CMD1H_DIR_INFO_READ   0x37           
#define    CMD0H_DIR_INFO_SAVE   0x38            
#define    CMD4H_BYTE_LOCATE     0x39            
#define    CMD2H_BYTE_READ       0x3A           
#define    CMD0H_BYTE_RD_GO      0x3B            
#define    CMD2H_BYTE_WRITE      0x3C            
#define    CMD0H_BYTE_WR_GO      0x3D            
#define    CMD0H_DISK_CAPACITY   0x3E           
#define    CMD0H_DISK_QUERY      0x3F            
#define    CMD0H_DIR_CREATE      0x40            
#define    CMD4H_SEC_LOCATE      0x4A           
#define    CMD1H_SEC_READ        0x4B           
#define    CMD1H_SEC_WRITE       0x4C            
#define    CMD0H_DISK_BOC_CMD    0x50            
#define    CMD5H_DISK_READ       0x54            
#define    CMD0H_DISK_RD_GO      0x55            
#define    CMD5H_DISK_WRITE      0x56            
#define    CMD0H_DISK_WR_GO      0x57            

#define    CMD_RET_SUCCESS       0x51            
#define    CMD_RET_ABORT         0x5F

#define    USB_INT_SUCCESS       0x14            
#define    USB_INT_CONNECT       0x15            
#define    USB_INT_DISCONNECT    0x16            
#define	   USB_INT_BUF_OVER	     0x17			
#define	   USB_INT_USB_READY	 	 0x18			
#define	   USB_INT_DISK_READ	 	 0x1D			
#define	   USB_INT_DISK_WRITE	 	 0x1E			
#define	   USB_INT_DISK_ERR	     0x1F	

#define	   ERR_DISK_DISCON			 0x82			
#define	   ERR_LARGE_SECTOR	     0x84			
#define	   ERR_TYPE_ERROR		 		 0x92			
#define	   ERR_BPB_ERROR	 		   0xA1			
#define	   ERR_DISK_FULL	    	 0xB1			
#define	   ERR_FDT_OVER		    	 0xB2			
#define	   ERR_FILE_CLOSE		 		 0xB4			
#define	   ERR_OPEN_DIR		       0x41			
#define	   ERR_MISS_FILE		 		 0x42		
#define	   ERR_FOUND_NAME		 		 0x43			
#define	   ERR_MISS_DIR		    	 0xB3			
#define	   ERR_LONG_BUF_OVER		 0x48			
#define	   ERR_LONG_NAME_ERR		 0x49			
#define	   ERR_NAME_EXIST		 		 0x4A	


/*******************************usb_Config*********************************/        
void usb_Config(void)
    {
			uint8_t respon=0;
			wr_ch376_low;
			rd_ch376_low;
			pcs_ch376_high;
			res_ch376_high;
			delay_ms(1000);
			res_ch376_low;
			
				
				scs_ch376_low;			
        usb_cmd_write(0,0x05);
				scs_ch376_high;
        delay_ms(35);

				
       while( respon != CMD_RET_SUCCESS)
				 {
					scs_ch376_low;
					usb_cmd_write(1,0x15,0x06);
					//usb_cmd_write(0,CMD0H_DISK_MOUNT);
					usb_read(1,&respon);
					scs_ch376_high;
					}


    } 

/*******************************usb_cmd***********************************/     
char usb_cmd_write(uint8_t len,uint8_t cmd,...)
    {
		va_list list;
		uint8_t buffer[20],i=1;
		
		buffer[0]=cmd;
		va_start(list,cmd);
		while(len--)
				{
				buffer[i]=va_arg(list,char);
				i++;
				}
			va_end(list);
			buffer[i]=0;
			len=0;
		while(len<i)
				{
				while(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_15))delay_us(1);	
				while(HAL_SPI_Transmit(&hspi2,&buffer[len],1,HAL_MAX_DELAY)!=HAL_OK);
				//delay_us(2);				
				len++;
				}
		//while(HAL_UART_Transmit(&huart1,buffer,i, HAL_MAX_DELAY)!=HAL_OK);
		return 0;
				
    }
	/*******************************usb_cmd_int***********************************/  	
	uint8_t usb_cmd_int(uint8_t len,uint8_t cmd)
{
		uint8_t status=0;
		char str[30];
		scs_ch376_low;
		usb_cmd_write(len,cmd);
		scs_ch376_high;
		//delay_ms(1);
		while(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_15))delay_us(1);	
		delay_us(3);
		scs_ch376_low;
		usb_cmd_write(0,CMD01_GET_STATUS);
		usb_read(1,&status);		
		scs_ch376_high;
		//sprintf(str,"cmd %x    return status %x\r\n",cmd , status);
		//debug(str);
		return status;
	}
/*******************************usb_read***********************************/ 	
char usb_read(uint8_t len,uint8_t *read_buffer)
		{
	//		uint8_t i=0,send=0xff;
			while(HAL_SPI_Receive(&hspi2,read_buffer,len,HAL_MAX_DELAY)!=HAL_OK);
			//while(HAL_UART_Transmit(&huart1,read_buffer,len, HAL_MAX_DELAY)!=HAL_OK);
	/*	while(len--)
				{	
				while(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_15));
				delay_us(2);				
				while(HAL_SPI_TransmitReceive(&hspi2,&send,&read_buffer[i++],1,HAL_MAX_DELAY)!=HAL_OK);
				while(HAL_UART_Transmit(&huart1,&read_buffer[i-1],1, HAL_MAX_DELAY)!=HAL_OK);
					
				}*/
		return 0;
		}
	
/*******************************usb_status***********************************/ 		
char usb_status(void)
    {
			
    uint8_t status=0;
		scs_ch376_low;
		usb_cmd_write(0,CMD01_GET_STATUS);
    usb_read(1,&status);
		scs_ch376_high;
		delay_ms(10);
		

		
    if(status == USB_INT_CONNECT || status == USB_INT_SUCCESS|| status == ERR_MISS_FILE)// ERR_MISS_FILE must folder close
        {
        debug_flash("<connected>\r\n");
        return 0;
        }
                            
    else
        {

        debug_flash("<disconnected>\r\n");
        return 1;
        } 
                        
    }
/*******************************usb_open**************************************/ 	
char usb_open(void) 
    {
    uint8_t status=0,timeout=10;
		
		status=usb_cmd_int(0,CMD0H_DISK_CONNECT);
    if(status ==  USB_INT_SUCCESS )
        {
				status=0;	
        while( status != USB_INT_SUCCESS && timeout)
						{
							status=usb_cmd_int(0,CMD0H_DISK_MOUNT);
							delay_ms(50);
							timeout--;							
						}
				
        if(status == USB_INT_SUCCESS )//USB_INT_SUCCESS
            {
            debug_flash("USB OPEN");
						return 0;
            }
        else
            {
            debug_flash("ERROR MOUNT\r\n");
            return 1;
            }
                            
        }
    else    
        {
        debug_flash("ERROR CONNECT\r\n");
        return 1;
        }
    }
/*******************************usb_open_fold**************************************/
char usb_open_fold(char *name) 
    {
      uint8_t status=0,timeout=10;
      
			do     
					{
					usb_set_name(name);
					delay_ms(50);
					timeout--;
					status=usb_cmd_int(0,CMD0H_FILE_OPEN);
//					char str[30];
//					sprintf(str,"usb_open_fold status=%d",status);
//					debug(str);
					}
					while(status != USB_INT_DISK_READ && status != ERR_OPEN_DIR  && timeout);
			if(timeout)
					{
					//usb_dir();
					return 0;
					} 
				debug_flash("timeout usb_open_fold()");
				return 1;
				
    }
/*******************************usb_set_name**************************************/ 	
void usb_set_name (char *name)
    {
    uint8_t str[30],i=1;

		str[0]=0x2f;
    while(*(name+i-1))
					{
					str[i]=*(name+i-1); 
					i++;
					}
		str[i++]=0;
		scs_ch376_low;
				while(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_15));
				//while(HAL_UART_Transmit(&huart1,str,i,HAL_MAX_DELAY)!=HAL_OK);				
				while(HAL_SPI_Transmit(&hspi2,str,i,HAL_MAX_DELAY)!=HAL_OK);
				scs_ch376_high;
				
    }
/*******************************usb_dir**************************************/ 
void usb_dir(void)
    {
		uint8_t status=0;
		
    debug_flash("<DIR>\r\n");

    do
				{
				uint8_t buffer[35]={0};
				scs_ch376_low;	
         usb_cmd_write(0,CMD01_RD_USB_DATA0);
				 delay_ms(1);
					usb_read(33,buffer);
					scs_ch376_high;
					
         send_file_properties(buffer);
				 //debug(buffer);
				 status=usb_cmd_int(0,CMD0H_FILE_ENUM_GO);
				 //delay_ms(10);
         }while(status != ERR_MISS_FILE);// || status==0x22); 
				
        debug_flash("</DIR>\r\n");
        usb_cmd_write(1,CMD1H_FILE_CLOSE,0);
        delay_ms(100);
   
    }
/*******************************usb_delet_file**********************************/ 
	
char usb_delet_folder(char *str)
	{
		uint8_t status=0,timeout=10;
	do{
		usb_set_name(str);
		delay_ms(50);
		status=usb_cmd_int(0,CMD0H_FILE_ERASE);
		timeout--;
		}	while(status != USB_INT_SUCCESS && timeout);
		if(status == USB_INT_SUCCESS)
				{
				debug_flash("usb_delet_folder:<deleted>\r\n");
				return 0;
				
				}
		else
				{
				debug_flash("usb_delet_folder:<error>\r\n");
				return 1;

				}
	}
/*******************************usb_delet_file**********************************/ 	
	char usb_delet_file(char *str)
	{
		uint8_t status=0,timeout=10;
	do{
		usb_set_name(str);
		delay_ms(50);
		status=usb_cmd_int(0,CMD0H_FILE_OPEN);
		timeout--;
		}	while(status != USB_INT_SUCCESS  && timeout);
		if(status == USB_INT_SUCCESS)
				{
					
				status=usb_cmd_int(0,CMD0H_FILE_ERASE);	
				debug_flash("usb_delet_file<deleted>\r\n");
				return 0;
				
				}
		else
				{
				debug_flash("usb_delet_file:<error>\r\n");
				return 1;

				}
	
	}
/*******************************send_file_properties**********************************/ 
    
void send_file_properties(uint8_t *properties_data)
    {
    char str[30],i,s,m,h,day,moun;
    unsigned int buffer,year;
    long int size=0;

    buffer=properties_data[16];
    buffer<<=8;
    buffer+=properties_data[15];
    h=buffer>>11;
    m=(buffer>>5) & 0x3f;
    s=(buffer<<1) & 0x3f;
    
    buffer=0;
    
    buffer=properties_data[18];
    buffer<<=8;
    buffer+=properties_data[17];
    year=(buffer>>9)+1980;
    moun=(buffer>>5) & 0x3f;
    day= buffer & 0x1f; 
    
    size = properties_data[29];
    size|=(long int)properties_data[30]<<8;
    size|=(long int)properties_data[31]<<16;
    size|=(long int)properties_data[32]<<24;
    
    i=0;
    str[0]='<';
		/*
		#define ATTR_READ_ONLY			0x01		
#define ATTR_HIDDEN				0x02		
#define ATTR_SYSTEM				0x04		
#define ATTR_VOLUME_ID			0x08	
#define ATTR_DIRECTORY			0x10		
#define ATTR_ARCHIVE			0x20	*/
    if(properties_data[12] ==   0x10 ||  properties_data[12] ==   0x16)
        
        str[1]='$';
		
		else if(properties_data[12] ==   0x0 )
        return;
    else
        str[1]='#';
           //32  
    for( i=0;i<12;i++)
        str[i+2]=properties_data[i+1];
        
    str[13]='>';
    str[14]=0;
    Send_Data_nrf(str);
		debug(str);
    sprintf(str,"<%ld|%d/%d/%d-%d:%d:%d>",size,year,moun,day,h,m,s);
		Send_Data_nrf(str);
		debug(str);
		delay_ms(10);
    }
/*******************************usb_download_file**********************************/ 	
void usb_download_file(char *str)
    {
    
    uint8_t status,buffer[5];
    char i[50];
		
    usb_set_name(str);
		status=usb_cmd_int(0,CMD0H_FILE_OPEN);
    if(status == USB_INT_SUCCESS)
        {
        long int size=0;
				scs_ch376_low;	
        usb_cmd_write(1,CMD14_GET_FILE_SIZE,0x68);
				delay_ms(1);
				usb_read(4,buffer);
				scs_ch376_high;
        size= buffer[0];
        size|=(long int)buffer[1]<<8;
        size|=(long int)buffer[2]<<16;
        size|=(long int)buffer[3]<<24;
        debug_flash("<start of file>\r\n");
				Send_Data_nrf("<start of file>\r\n");
        sprintf(i,"<size'%ld'size>\r\n",size);
        debug(i);
				Send_Data_nrf(i);
        delay_ms(100);
        
				CS_LOW; 
				Opr_Mode = 0; 
				Command_Reg = 0x4E;
				Set_Reg(W_REGISTER);   
				send_actived = 1;
				delay_ms(10);
				State=1;
				HAL_TIM_Base_Start_IT(&htim1);
        while(size>0)
            {

						scs_ch376_low;
						usb_cmd_write(2,CMD2H_BYTE_READ,0xff,0xff);
						scs_ch376_high;
						delay_ms(1);
						while(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_15))delay_us(1);		
						delay_us(3);
						scs_ch376_low;
						usb_cmd_write(0,CMD01_GET_STATUS);
						usb_read(1,&status); 
						scs_ch376_high;
						
            if(status == USB_INT_DISK_READ) 
                {

                while(1)
                    {
                    
                    if(status != USB_INT_SUCCESS )
                        {
												uint8_t send_pack_buffer[255],read_size;
												char str[40];
												scs_ch376_low;	
												usb_cmd_write(0,CMD01_RD_USB_DATA0);
												//delay_ms(1);
												usb_read(1,&read_size);
												//sprintf(str,"read_size %x\r\n", read_size);
												//debug(str);
												//while(HAL_UART_Transmit(&huart1,send_pack,1, HAL_MAX_DELAY)!=HAL_OK);
												usb_read(read_size,send_pack_buffer);
												//while(HAL_UART_Transmit(&huart1,send_pack_buffer,read_size, HAL_MAX_DELAY)!=HAL_OK);
												scs_ch376_high;
												
												while(save_downloade(read_size,send_pack_buffer) == 0)delay_us(1);
												//debug((char *)send_pack_buffer);
												//send_pack(read_size,send_pack_buffer);
												if(error_nrf >10 )
															{
															debug_flash("error_nrf >10\r\n");
															HAL_TIM_Base_Stop_IT(&htim1);
																error_nrf=0;
																nRF_Config();
																CS_LOW; 
																Opr_Mode = 0; 
																Command_Reg = 0x4E;
																Set_Reg(W_REGISTER);   
																send_actived = 1;
																delay_ms(10);
																State=1;
																HAL_TIM_Base_Start_IT(&htim1);
															}
                        }
                    else
                        break;
                    status=usb_cmd_int(0,CMD0H_BYTE_RD_GO);
                    }
                }
            size-=0xffff;              
            }
				while(save_downloade(16,"</end of file>\r\n")== 0)delay_us(1);					
				usb_cmd_write(1,CMD1H_FILE_CLOSE,1);
        //delay_ms(100);
				while(downloade_cun !=send_cun || downloade_flag==1)
				{delay_ms(50);
				debug_number("downloade_cun",downloade_cun);
				debug_number("send_cun",send_cun);	
				if(error_nrf >10 )
							{
							debug_flash("error_nrf >10\r\n");
							HAL_TIM_Base_Stop_IT(&htim1);
								error_nrf=0;
								nRF_Config();
								CS_LOW; 
								Opr_Mode = 0; 
								Command_Reg = 0x4E;
								Set_Reg(W_REGISTER);   
								send_actived = 1;
								delay_ms(10);
								State=1;
								HAL_TIM_Base_Start_IT(&htim1);
							}				
				}

				debug_number("downloade_cun",downloade_cun);
				debug_number("send_cun",send_cun);						
				debug_flash("</end of file>\r\n");
			
				HAL_TIM_Base_Stop_IT(&htim1);
				downloade_cun=0;
				send_cun=0;
				timer_cun=0;
				State=0;
				downloade_flag=0;
				nRF_Config();				
				delay_ms(100);
				//Send_Data_nrf("</end of file>\r\n");

        }
				else
				debug_flash("can not open file for download\r\n   "); 
				
				
       // CS_HIGH;
				
			}
/*******************************send_pack**********************************/ 	
void send_pack(void)
    {
//    unsigned char k=0,j=0,ack=0;
//    unsigned int i=0; 
//		char payload1[40];	
//    
//    i=tx_size;
//    if(i<150)
//        delay_ms(1);
//    if(i<100)
//        delay_ms(1);
//            
//    while(i>29)
//        {
////        payload1[0]=32;
//        payload1[0]='!';
//        payload1[1]=ack;
////        
//        for(j=0;j<30;j++)
//            payload1[j+2]=buffer[k++];
//            ack++;
////                do{
////                
//////                Command_Reg = 0x7E; 
//////                Set_Reg(0x27); 
////                Set_Reg(FLUSH_TX); 
////                Set_Reg(W_TX_PAYLOAD);
////                State=0;           
////                CS_HIGH;
////                delay_us(20);
////                CS_LOW;
////                while(!State);    
////                }while(State !=2);   
//				Send_Data(31,payload1);
//        i-=30;
//        }

//  //  payload1[0]=i+2;
//    payload1[0]='!';
//    payload1[1]=ack;    
//    for(j=0;j<i;j++)
//            payload1[j+2]=buffer[k++];
////        do{

////        Set_Reg(FLUSH_TX); 
////        Set_Reg(W_TX_PAYLOAD);
////        State=0;           
////        CS_HIGH;
////        delay_us(20);
////        CS_LOW;
////        while(!State);
////        }while(State !=2);    
//      Send_Data(i+2,payload1); 
	if(downloade_cun == 0 && downloade_flag==0)
		return;
	if(timer_cun == 1)
		{			
		if(State==2)
				{
				int i=0;
				State=0;				
				while (((downloade_cun > send_cun && downloade_flag==0 ) || (send_cun > downloade_cun && downloade_flag==1)) && i<30)
							{
							payload[i+1]=downloade_buffer[send_cun++];
							if(send_cun==9999)
									{
									send_cun=0;
									downloade_flag=0;
									}
									i++;
							}
				payload[0]=i+1;
				//payload[1]='!';							
				}
		}
 else if(timer_cun == 2)
		{	 
		if((Temp_Addrs[4]==Base_Addrs[4]) && (Temp_Addrs[3]==Base_Addrs[3]) && (Temp_Addrs[2]==Base_Addrs[2]) && (Temp_Addrs[1]==Base_Addrs[1]) && (Temp_Addrs[0]==Base_Addrs[0]))
        {
					//debug(payload);
					//				payload[0]=30;
				//payload[1]='!';
            Set_Reg(FLUSH_TX); 
            Set_Reg(W_TX_PAYLOAD);           
           // delay_us(10);
            CS_HIGH;
						//delay_us(20);
						//CS_LOW;
				}
		}
	else if(timer_cun ==5)
		CS_LOW;
else if(timer_cun ==800 || State == 2)	
		{
		if(timer_cun == 800)
		{
		timer_cun=1;
		debug_number("timer_cun",timer_cun);
		error_nrf++;
//		if(error_nrf >10 )
//			{
//				error_nrf=0;
//				nRF_Config();
//				CS_LOW; 
//				Opr_Mode = 0; 
//				Command_Reg = 0x4E;
//				Set_Reg(W_REGISTER);   
//				send_actived = 1;
//				
//			}
		
		}
		else 
			timer_cun=0;
		}
	timer_cun++;
	
    }
/*******************************save_downloade**********************************/ 			
uint8_t save_downloade(uint8_t size,uint8_t *read_buffer)
	{

	if((downloade_cun >= send_cun && downloade_flag==0) || (send_cun > downloade_cun && downloade_flag==1))
		{
		for(int i=0;i<size;i++)
				{
				downloade_buffer[downloade_cun++]=*read_buffer++;
				while(downloade_cun+300 > send_cun  && downloade_flag == 1)
				{delay_ms(10);if(error_nrf >10 )
															{
															debug_flash("error_nrf >10\r\n");
															HAL_TIM_Base_Stop_IT(&htim1);
																error_nrf=0;
																nRF_Config();
																CS_LOW; 
																Opr_Mode = 0; 
																Command_Reg = 0x4E;
																Set_Reg(W_REGISTER);   
																send_actived = 1;
																delay_ms(10);
																State=1;
																HAL_TIM_Base_Start_IT(&htim1);
															}}
				if(downloade_cun == 9999)
						{	
						downloade_cun=0;
						downloade_flag=1;
						//debug_flash("flage=1");
						}
				}
			return 1;
		}
	debug_flash("download stop");
	debug_number("downloade_cun",downloade_cun);
	debug_number("send_cun",send_cun);
	debug_number("downloade_flag",downloade_flag);
	return 0;

	}
		