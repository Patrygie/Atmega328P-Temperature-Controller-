#define F_CPU 16000000UL

#include <LCD_I2C.h>
#include <util/delay.h>
#include <math.h>

#define LCD_HEIGHT 2
#define LCD_WIDTH 16
#define MEM_ADDR 22
#define TEMP_LOW 30
#define TEMP_HIGH 59
#define CONTROL_CODE_WRITE 0xA0
#define CONTROL_CODE_READ 0xA1

LCD_I2C Lcd(0x27, LCD_WIDTH, LCD_HEIGHT);
void LCD_ShowPage(uint8_t page);

void ADC_PWM_Disp(int, int);
void Temp_Max_Disp(float, float);
void EEPROM_Disp(uint8_t, uint8_t, uint8_t);

void ADC_Start(void);
uint16_t ADC_Read();

void PWM_Start(void);

void I2C_Init(void);
void I2C_Start(void);
void I2C_Stop(void);
uint8_t I2C_Write(uint8_t data);
uint8_t I2C_ReadAck();
void I2C_WriteToAdress(uint8_t control_code_write, uint16_t eeprom_adress, uint8_t data_to_send);
uint8_t I2C_ReadFromAdress(uint8_t control_code_write, uint8_t control_code_read, uint16_t mem_adress);

uint16_t adc_data = 0;
float VOLT = 0;
float TEMP = 0;
float temporary = 0;
float max_temp = 0;
uint8_t page = 0;
uint8_t addr = MEM_ADDR;
uint8_t data_r = 0;
uint8_t data_w = 0;

void setup()
{
  Lcd.begin();
  Lcd.backlight();

  ADC_Start();
  PWM_Start();
  I2C_Init();

  //Pin direction
  DDRD |= (1<<PD5) | (1<<PD6);
  DDRD |= (1<<PD7); 

  //Pin state
  PORTD |= (1<<PD7) | (1<<PD2);
} 

void loop() 
{
  Lcd.clear();
  LCD_ShowPage(page);
  
  if(!(PIND & (1<<PD2)))
  {
    while(!(PIND & (1<<PD2)));
    page++;
    if(page == 3) page = 0;
  }

  PORTD ^= (1<<PD5);

  adc_data = ADC_Read();
  
  VOLT = (adc_data * 5.0) / 1024.0;
  TEMP = VOLT  * 100;
  temporary = TEMP;

  if(TEMP <= TEMP_LOW)
  {
    OCR0A = 0;
  }
  else if(TEMP > TEMP_LOW && TEMP <= TEMP_HIGH)
  {
    OCR0A = TEMP*TEMP / 14;
  }
  else 
  {
    OCR0A = 255;
  }

  if(max_temp <= temporary)
  {
    max_temp = temporary;
    data_w = max_temp;

    I2C_WriteToAdress(CONTROL_CODE_WRITE, MEM_ADDR, data_w);
    _delay_ms(5); //necessary delay for memory
    data_r = I2C_ReadFromAdress(CONTROL_CODE_WRITE, CONTROL_CODE_READ, MEM_ADDR);
  }

  _delay_ms(20);
}

//------------------------- LCD --------------------------
void ADC_PWM_Disp(int adc, int pwm)
{
  Lcd.setCursor(0, 0);
  Lcd.print("1.   SIGNALS");
  
  Lcd.setCursor(0, 1);
  Lcd.print("ADC:");
  Lcd.setCursor(4, 1);
  Lcd.print(adc);

  Lcd.setCursor(9, 1);
  Lcd.print("PWM: ");
  Lcd.setCursor(13, 1);
  Lcd.print(pwm);
}

void Temp_Max_Disp(float temp, float max)
{
  Lcd.setCursor(0, 0);
  Lcd.print("2.  MEASURES");

  Lcd.setCursor(0, 1);
  Lcd.print("T: ");
  Lcd.setCursor(2, 1);
  Lcd.print(temp);
  
  Lcd.setCursor(8, 1);
  Lcd.print("M: ");
  Lcd.setCursor(10, 1);
  Lcd.print(max);
}

void EEPROM_Disp(uint8_t addr, uint8_t data_w, uint8_t data_r)
{
  Lcd.setCursor(0, 0);
  Lcd.print("3.EEPROM");

  Lcd.setCursor(0, 1);
  Lcd.print("ADDR: ");
  Lcd.setCursor(5, 1);
  Lcd.print(addr);

  Lcd.setCursor(9, 0);
  Lcd.print("DW: ");
  Lcd.setCursor(12, 0);
  Lcd.print(data_w);

  Lcd.setCursor(9, 1);
  Lcd.print("DR: ");
  Lcd.setCursor(12, 1);
  Lcd.print(data_r);
}

void LCD_ShowPage(uint8_t page)
{
switch(page)
  {
    case 0: ADC_PWM_Disp(adc_data, OCR0A);
      break;
    case 1: Temp_Max_Disp(TEMP, max_temp);
      break;
    case 2: EEPROM_Disp(addr, data_w, data_r);
      break;
    default:
      Lcd.clear();
  }
}
//--------------------------------------------------------

//------------------------- ADC --------------------------
void ADC_Start(void)
{
  ADMUX = (1<<REFS0);
  ADCSRA = (1<<ADEN) | (1<<ADSC) | (1<<ADPS2);
}

uint16_t ADC_Read()
{
  ADCSRA |= (1<<ADSC);
  while(ADCSRA& (1<<ADSC));
  return ADC;
}
//--------------------------------------------------------


//------------------------- PWM --------------------------
void PWM_Start(void)
{
  TCCR0A |= (1<<WGM00) | (1<<WGM01) | (1<<COM0A1);
  TCCR0B |= (1<<CS01); 
}
//--------------------------------------------------------


//------------------------- I2C --------------------------
void I2C_Init(void)
{
  TWSR = 0;
  TWBR = ((F_CPU/100000)-16) / (2*pow(4, (TWSR&( (1<<TWPS0) | (1<<TWPS1)))));
}

void I2C_Start(void)
{
  TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);
  while (!(TWCR & (1<<TWINT)));
}

void I2C_Stop(void)
{
  TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWSTO);
  while (!(TWCR & (1<<TWSTO)));
}

uint8_t I2C_Write(uint8_t data)
{
  TWDR = data;
  TWCR = (1<<TWINT) | (1<<TWEN);
  while (!(TWCR & (1<<TWINT)));
}

uint8_t I2C_ReadAck()
{
  TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);
  while (!(TWCR & (1<<TWINT)));
  return TWDR;
}

void I2C_WriteToAdress(uint8_t control_code_write, uint16_t mem_adress, uint8_t data_to_send)
{
  I2C_Start();
  I2C_Write(control_code_write);
  I2C_Write(mem_adress);
  I2C_Write(data_to_send);
  I2C_Stop();  
}

uint8_t I2C_ReadFromAdress(uint8_t control_code_write, uint8_t control_code_read, uint16_t mem_adress)
{
  I2C_Start();
  I2C_Write(control_code_write);
  I2C_Write(mem_adress);

  I2C_Start();
  I2C_Write(control_code_read);
  uint8_t data_readed = I2C_ReadAck();
  I2C_Stop();

  return data_readed;
}
//------------------------------------------------------


