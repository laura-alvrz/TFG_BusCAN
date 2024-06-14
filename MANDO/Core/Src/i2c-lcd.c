
/** Put this in the src folder **/

#include "i2c-lcd.h"
extern I2C_HandleTypeDef hi2c1;  // change your handler here accordingly

#define SLAVE_ADDRESS_LCD 0x4E // change this according to ur setup

#define LINES 2 //Dimensiones LCD
#define ROWS 16 //Dimensiones LCD

void lcd_send_data (char data){
	char data_u, data_l;
	uint8_t data_t[4];
	data_u = (data&0xf0);
	data_l = ((data<<4)&0xf0);
	data_t[0] = data_u|0x0D;  //en=1, rs=1
	data_t[1] = data_u|0x09;  //en=0, rs=1
	data_t[2] = data_l|0x0D;  //en=1, rs=1
	data_t[3] = data_l|0x09;  //en=0, rs=1
	HAL_I2C_Master_Transmit (&hi2c1, SLAVE_ADDRESS_LCD,(uint8_t *) data_t, 4, HAL_MAX_DELAY);
}

void lcd_send_cmd (char cmd){
  char data_u, data_l;
	uint8_t data_t[4];
	data_u = (cmd&0xf0);
	data_l = ((cmd<<4)&0xf0);
	data_t[0] = data_u|0x0C;  //en=1, rs=0
	data_t[1] = data_u|0x08;  //en=0, rs=0
	data_t[2] = data_l|0x0C;  //en=1, rs=0
	data_t[3] = data_l|0x08;  //en=0, rs=0
	HAL_I2C_Master_Transmit (&hi2c1, SLAVE_ADDRESS_LCD,(uint8_t *) data_t, 4, HAL_MAX_DELAY);
}

void lcd_init (void){
	HAL_Delay(45); //Datasheet especifica espera de 40ms
	lcd_send_cmd (0x30); //Encendido del display y del cursor
	HAL_Delay(5); //Datasheet especifica espera de 4.1ms
	lcd_send_cmd (0x30); //El datasheet especifica que se debe repetir el envio de esta instruccion
	HAL_Delay(1); //Datasheet especifica espera de 100us
	lcd_send_cmd(0x30);
	HAL_Delay(1);
	lcd_send_cmd (0x20);  // 4bit mode
	HAL_Delay(1);
	lcd_send_cmd (0x28); // Function set --> DL=0 (4 bit mode), N = 1 (2 line display) F = 0 (5x8 characters)
	HAL_Delay(1);
	lcd_send_cmd (0x08); //Display on/off control --> D=0,C=0, B=0  ---> display off
	HAL_Delay(1);
	lcd_send_cmd (0x01);  //Clear
	HAL_Delay(1);
	lcd_send_cmd (0x06); //Entry mode set --> I/D = 1 (increment cursor) & S = 0 (no shift)
	HAL_Delay(1);
	lcd_send_cmd (0x0C); //Display on/off control --> D = 1, C and B = 0. (Cursor and blink, last two bits)
	HAL_Delay(1);
}

void lcd_send_string (char *str){
	//Escribe a continuación de lo que haya
	static int contador_linea = 0; //Static para que no se borre entre ejecuciones de la función
	contador_linea = 0;
	while (*str){
		lcd_send_data (*str++);
		contador_linea = (contador_linea) % (ROWS*LINES); //Contar hasta el número de caracteres máximo
		contador_linea++;
		if (contador_linea == ROWS)
			lcd_send_cmd(0xc0); //Pasa a la segunda línea
		if (contador_linea > ROWS*LINES)
			return; //No hace nada más porque no hay espacio
	}
}

void lcd_update (char *str){

	lcd_clear(); //Borra la pantalla y pone el cursor en la esquina superior izquierda
	lcd_send_string(str);

}

void lcd_clear(void){ //Tambien posiciona el cursor en la esquina superior izquierda. SOLO VALE PARA DISPLAY 16x2
	lcd_send_cmd(0x80); //Posiciona en el comienzo
	for (int i=0; i < LINES; i++){ //Recorrer todas las filas
		for (int j=0; j < ROWS; j++) //Recorrer todas las columnas
			lcd_send_data(' '); //Poner espacios (borrar todo)
		lcd_send_cmd(0x0c); //Posicionar en la segunda linea
	}
	lcd_send_cmd(0x80); //Posicionar el cursor en el comienzo de nuevo
}

/*
void lcd_send_cmd (char cmd){
  char data_u, data_l;
	uint8_t data_t[4];
	data_u = (cmd&0xf0);
	data_l = ((cmd<<4)&0xf0);
	data_t[0] = data_u|0x0C;  //en=1, rs=0
	data_t[1] = data_u|0x08;  //en=0, rs=0
	data_t[2] = data_l|0x0C;  //en=1, rs=0
	data_t[3] = data_l|0x08;  //en=0, rs=0
	HAL_I2C_Master_Transmit (&hi2c1, SLAVE_ADDRESS_LCD,(uint8_t *) data_t, 4, 100);
}

void lcd_send_data (char data){
	char data_u, data_l;
	uint8_t data_t[4];
	data_u = (data&0xf0);
	data_l = ((data<<4)&0xf0);
	data_t[0] = data_u|0x0D;  //en=1, rs=0
	data_t[1] = data_u|0x09;  //en=0, rs=0
	data_t[2] = data_l|0x0D;  //en=1, rs=0
	data_t[3] = data_l|0x09;  //en=0, rs=0
	HAL_I2C_Master_Transmit (&hi2c1, SLAVE_ADDRESS_LCD,(uint8_t *) data_t, 4, 100);
}*/

/*void lcd_enviar(char *string,int row,int col){
	lcd_put_cur(row,col);
	lcd_send_string(string);
}*/

/*void lcd_clear (void){
	lcd_send_cmd (0x80);
	for (int i=0; i<70; i++)
	{
		lcd_send_data (' ');
	}
}

void lcd_put_cur(int row, int col){
    switch (row){
        case 0:
            col |= 0x80;
            break;
        case 1:
            col |= 0xC0;
            break;
    }

    lcd_send_cmd (col);
}


void lcd_init (void)
{
	// 4 bit initialisation
	HAL_Delay(50);  // wait for >40ms
	lcd_send_cmd (0x30);
	HAL_Delay(5);  // wait for >4.1ms
	lcd_send_cmd (0x30);
	HAL_Delay(1);  // wait for >100us
	lcd_send_cmd (0x30);
	HAL_Delay(10);
	lcd_send_cmd (0x20);  // 4bit mode
	HAL_Delay(10);

  // dislay initialisation
	lcd_send_cmd (0x28); // Function set --> DL=0 (4 bit mode), N = 1 (2 line display) F = 0 (5x8 characters)
	HAL_Delay(1);
	lcd_send_cmd (0x08); //Display on/off control --> D=0,C=0, B=0  ---> display off
	HAL_Delay(1);
	lcd_send_cmd (0x01);  // clear display
	HAL_Delay(1);
	HAL_Delay(1);
	lcd_send_cmd (0x06); //Entry mode set --> I/D = 1 (increment cursor) & S = 0 (no shift)
	HAL_Delay(1);
	lcd_send_cmd (0x0C); //Display on/off control --> D = 1, C and B = 0. (Cursor and blink, last two bits)
}

void lcd_send_string (char *str)
{
	while (*str) lcd_send_data (*str++);
}

*/
