#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "freertos/timers.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "stdlib.h"
#include "freertos/queue.h"
#include <math.h>

//***************************************************************DEFINICION DE VARIABLES***************************************
// Definimos los pines de los LEDS
#define ledBlink        2
#define ledR            33
#define ledG            25
#define ledB            26
#define ledControl      32

//Definicion de ISR (botones)
#define button_SUM  22
#define button_RES 23

//Definiciones del UART
#define UART_NUM UART_NUM_0
#define BUF_SIZE 1024 * 2
#define TASK_MEMORY 1024 *2
#define PIN_TX 1
#define PIN_RX 3

#define lenght_Rx 13

//Definiciones del DC
#define SUPPLY_VOLTAGE  3.3
#define BITS_ADC_CONFI  4096

//Definimos comandos ON Y OFF de los LEDs
#define LEDR_ON         gpio_set_level(ledR,1);
#define LEDG_ON         gpio_set_level(ledG,1);
#define LEDB_ON         gpio_set_level(ledB,1);
#define LED_BLINK_ON    gpio_set_level(ledBlink,1);
#define LEDCONTROL_ON   gpio_set_level(ledControl,1);

#define LEDR_OFF         gpio_set_level(ledR,0);
#define LEDG_OFF         gpio_set_level(ledG,0);
#define LEDB_OFF         gpio_set_level(ledB,0);
#define LED_BLINK_OFF    gpio_set_level(ledBlink,0);
#define LEDCONTROL_OFF   gpio_set_level(ledControl,0);

//static const char *tag = "Main";

//***************************************************************PROTOTIPADO DE VARIABLES***************************************
//Variables TASK
typedef struct { // Definición de la estructura que contendrá los datos
    int identifier;
    float data;
} Temperatures;


QueueHandle_t queue; // Creación de la cola para enviar los datos



//Variables de INTERRUPCIONES
int banderaSUM = 0;
int banderaRES = 0;


//Variables del UART
static QueueHandle_t uart_queue;
uint8_t transmit_text[200];

uint8_t len;
char message_to_send[100];

uint8_t pos_in;
uint8_t pos_fin;

int threshold_temp = 30;
int hist_temp = 25;//40-10;


//Variables del ADC
int raw = 0;
int res_2 = 1000;
float temp = 1.0;
float v_res = 0.0;
float res_ntc = 0.0;
int temp_index = 0;

float valores_res_ntc[] = {3210,2830,2500,2210,1960,1750,1560, 1390, 1240, 1110,1000, 899, 810, 731, 661, 599, 543, 493, 449, 409, 373, 341, 312, 286, 262,241,221,204,188,174,160,148,137,127,118,110,102,95.1,88.6,82.7,79.3};
float valores_temp_ntc[] = {0,2.5,5,7.5,10,12.5,15,17.5,20,22.5,25,27.5,30,32.5,35,37.5,40,42.5,45,47.5,50,52.5,55,57.5,60,62.5,65,67.5,70,72.5,75,77.5,80,82.5,85,87.5,90,92.5,95,97.5,99};

//***************************************************************PROTOTIPADO DE FUNCIONES***************************************
//Prototipado de funciones de los LEDs
esp_err_t init_led(void);

//Prototipado de funciones del ADC
int encontrar_posicion_mas_cercana(float* vector, int longitud, float variable);


//Prototipado de funciones del UART
static int Get_number(char *data);
static void print_Hist(void);
static void print_Threshold(void);

//Prototipado de funciones de INTERRUPCIONES
void isr_handlerSUM(void *args);
void isr_handlerRES(void *args);


//***************************************************************INICIALIZACIÓN de PERIFÉRICOS***************************************
esp_err_t init_led(void) 
{
    // Inicializo el pin del led ROJO
    gpio_reset_pin(ledR);
    gpio_set_direction(ledR, GPIO_MODE_OUTPUT);

    // Inicializo el pin del led VERDE
    gpio_reset_pin(ledG);
    gpio_set_direction(ledG, GPIO_MODE_OUTPUT);

    // Inicializo el pin del led AZUL
    gpio_reset_pin(ledB);
    gpio_set_direction(ledB, GPIO_MODE_OUTPUT);


    // Inicializo el pin del led BLINK
    gpio_reset_pin(ledBlink);
    gpio_set_direction(ledBlink, GPIO_MODE_OUTPUT);

    // Inicializo el pin del led CONTROL
    gpio_reset_pin(ledControl);
    gpio_set_direction(ledControl, GPIO_MODE_OUTPUT);
    LEDCONTROL_OFF;

    return ESP_OK;
}

// Función de inicialización del ADC
esp_err_t init_ADC(void){
    adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_11); //GPIO Vp
    adc1_config_width(ADC_WIDTH_BIT_12); //4096
    return ESP_OK;
}

// Función de inicialización de INTERRUPCIONES
esp_err_t init_isr(void)
{
    gpio_config_t pISRConfig;
    pISRConfig.pin_bit_mask = ((1ULL << button_SUM)|(1ULL << button_RES));
    pISRConfig.mode = GPIO_MODE_DEF_INPUT;
    pISRConfig.pull_down_en = GPIO_PULLDOWN_DISABLE;
    pISRConfig.pull_up_en = GPIO_PULLUP_ENABLE;
    pISRConfig.intr_type = GPIO_INTR_NEGEDGE;
    
    gpio_config(&pISRConfig); 
    gpio_install_isr_service(0);
    gpio_isr_handler_add(button_SUM,isr_handlerSUM,NULL);
    gpio_isr_handler_add(button_RES,isr_handlerRES,NULL);
    return ESP_OK;
}

// Función de inicialización del UART
static void init_uart(void)
{
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        //.rx_flow_ctrl_thresh = 122,
        .source_clk = UART_SCLK_APB,
    };

    uart_param_config(UART_NUM, &uart_config);
    uart_set_pin(UART_NUM, PIN_TX, PIN_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM,BUF_SIZE,BUF_SIZE,20,&uart_queue,0);
    //xTaskCreate(uart_task, "uart_event_task", TASK_MEMORY,NULL,12,NULL);
}




//***********************************************************************TASK PARA ENVIAR Y RECIBIR************************************/

//***********************************************************************TASK DEL ADC*************************************************/
void Get_prom_temp(void *pvParameter)
{
    while(true){
        Temperatures temperas;
        float prom_temp = 0;

        for(int i = 0; i < 10; i++)
        {
            raw = adc1_get_raw(ADC1_CHANNEL_0);
            v_res = (raw * SUPPLY_VOLTAGE) / BITS_ADC_CONFI;
            res_ntc = (SUPPLY_VOLTAGE / v_res - 1)* res_2;

            temp_index = encontrar_posicion_mas_cercana(valores_res_ntc, sizeof(valores_res_ntc) / sizeof(valores_res_ntc[0]), res_ntc);
            temp = valores_temp_ntc[temp_index];

            prom_temp += temp;
            vTaskDelay( 100 / portTICK_RATE_MS);
        }

        temperas.data = prom_temp/10;
        temperas.identifier = 0;
        //printf ("LA TEMPERATURA ARRIBA ES: %.2f \n",temperas.data);
        xQueueSend( queue, &temperas, pdMS_TO_TICKS(0) );
        //printf("SE ENVIO LA COLA \n");
        vTaskDelay( 4000 / portTICK_PERIOD_MS);
    }
}

//***********************************************************************TASK DEL UART*************************************************/
void task_uart (void *pvParameter)
{
    Temperatures temperas = {0,0};
    while(1){
        /*************************************************************PROGRAMACIÓN UART**********************************************************************/
        const char* data = (const char *)malloc(BUF_SIZE);
        uart_event_t event;

        if (xQueueReceive(uart_queue,(void *)&event,pdMS_TO_TICKS(100))){ //Si recibió un evento de UART
            bzero( data , BUF_SIZE);
            switch (event.type){
            case UART_DATA: //Si recibió un mensaje
                uart_read_bytes(UART_NUM, data, event.size,pdMS_TO_TICKS(100));
                //uart_write_bytes(UART_NUM,(const char *)data,event.size);
                if(strstr(data,"SET_TEMP") != 0){
                    threshold_temp = Get_number(data);
                    temperas.data = threshold_temp;
                    temperas.identifier = 1;
                }

                if(strstr(data,"SET_HIST") != 0){
                    hist_temp = Get_number(data);
                    temperas.data = hist_temp;
                    temperas.identifier = 2;
                }

                if(strstr(data,"GET_TEMP") != 0){
                    print_Threshold();
                }

                if(strstr(data,"GET_HIST") != 0){
                    print_Hist();
                }

                uart_flush(UART_NUM); //Asegura que todo el DATA está transmitido antes de que pase a la siguiente línea
                break;
            default:
                break;
            }
        }

        //printf ("EL THRESHOLD EN COLAS ES: %.2f \n",temperas.threshold_temp);
        //printf ("LA HISTERESIS EN COLAS ES: %.2f \n",temperas.hist_temp);
        //Enviamos la estructura a la cola
        xQueueSend( queue, &temperas, pdMS_TO_TICKS(0) );
        //printf("SE ENVIO LA COLA \n");
        vTaskDelay(5000 / portTICK_RATE_MS); // Esperar un segundo para volver a medir la temperatura //5000
    }
}

void task_increase_control (void *pvParameters) 
{
    Temperatures temperas;
    //int cont = 0;

    float prom_temp = 0;
    int threshold_temp = 0;
    int hist_temp = 0;
    while (1) 
    {
        //cont = 0;
        //Esperamos a recibir los datos de la cola

        while ( xQueueReceive(queue, &temperas, pdMS_TO_TICKS(1000)) == true ){
            //printf("COLA RECIBIDA Y EL DATO ES: %.2f y SU TIPO ES: %d\n",temperas.data, temperas.identifier);
            //cont ++;
            //printf ("CONTADOR = %d\n", cont);
            //printf("COLA RECIBIDA Y EL THRESHOLD ES: %.2f \n",temperas.data);
            //printf("COLA RECIBIDA Y LA HISTERESIS ES: %.2f \n",temperas.data);
        }

        if (temperas.identifier == 0){
            prom_temp = temperas.data;
        }

        if (temperas.identifier == 1){
            threshold_temp = temperas.data;
        }

        if (temperas.identifier == 2){
            hist_temp = temperas.data;
        }

        printf("La TEMPERATURA ES: %.2f EL THRESHOLD ES: %d Y LA HISTERESIS ES: %d \n", prom_temp, threshold_temp,hist_temp);
        // Si la temperatura está entre hist_temp y threshold_temp, el LED permanece en su estado anterior
        if (prom_temp <= hist_temp) {
            //printf("LEDCONTROL ON y la TEMPERATURA ES: %.2f\n", prom_temp);
            LEDCONTROL_ON;
        }
        else if (prom_temp >= threshold_temp) {
            //printf("LEDCONTROL OFF y la TEMPERATURA ES: %.2f\n", prom_temp);
            LEDCONTROL_OFF;
        }


        if ( prom_temp < hist_temp) {
            LEDB_ON;
            LEDR_OFF;
            LEDG_OFF;
        }

        else if (prom_temp > threshold_temp){
            LEDR_ON;
            LEDB_OFF;
            LEDG_OFF;
        }
        else{
            LEDG_ON;
            LEDB_OFF;
            LEDR_OFF;
        }

        //printf ("La temperatura es: %.2f\n", prom_temp);
        vTaskDelay(5000/ portTICK_RATE_MS); //5000
    }
}

void task_control_threshold_button(void *pvParameters)
{
    Temperatures temperas;
    float threshold_button = 0;
    float hist_button = 0;
    while(1)
    {
        //Esperamos a recibir los datos de la cola
        while ( xQueueReceive(queue, &temperas, pdMS_TO_TICKS(1000)) == true )
        {
           //printf("COLA RECIBIDA Y EL THRESHOLD ES: %.2f \n",temperas.data);
           //printf("COLA RECIBIDA Y LA HISTERESIS ES: %.2f \n",temperas.data);
           //printf("COLA RECIBIDA Y EL THRESHOLD BUTTON ES: %.2f \n",temperas.data);
        }

        if (temperas.identifier == 1 ){
            threshold_button = temperas.data;
        }

        if (temperas.identifier == 2 ){
            hist_button = temperas.data;
        }

        //printf ("EL THRESHOLD BUTTON ES: %.2f\n",threshold_button);
        //printf ("LA HISTERESISI BUTTON ES: %.2f\n",hist_button);
        
        if(banderaSUM == 1){

            if ( (threshold_button +5) <= 99 ){
                threshold_button = threshold_button +5;
            }

            if (hist_button > threshold_button){
                hist_button = threshold_button;
                temperas.data = hist_button;
                temperas.identifier = 2;
            }
            banderaSUM = 0;
        }

        if (banderaRES == 1){
            if ( (threshold_button -5) >= 0 ){
                threshold_button = threshold_button -5;
            }

            if (hist_button > threshold_button){
                hist_button = threshold_button;
                temperas.data = hist_button;
                temperas.identifier = 2;

            }
            banderaRES = 0;
        }

        temperas.data = threshold_button;
        temperas.identifier = 1;

        xQueueSend( queue, &temperas, pdMS_TO_TICKS(0) );


        //printf("EL THRESHOLD CON BOTON ES: %.2f \n",threshold_button);
        //printf("LA HISTERESIS CON BOTON ES: %.2f \n",hist_button);
        vTaskDelay( 5000 / portTICK_RATE_MS ); //6000 //5000
    }
}


void app_main(void)
{
    init_led();
    init_isr();
    init_ADC();
    init_uart();

    queue = xQueueCreate(10, sizeof(Temperatures)); //Creamos la cola

    //*************************************************TASK*****************************************************************************/

    /**********************************PROGRAMACIÓN OBTENER TEMPERATURA DE LA NTC (ADC)*********************************/
    xTaskCreate(&Get_prom_temp, "Get_prom_temp", 2048, NULL, 1, NULL);

    /********************************************PROGRAMACIÓN UART******************************************************/
    xTaskCreate(&task_uart, "task_uart", 2048, NULL, 1, NULL);
    xTaskCreate(&task_increase_control, "task_increase_control", 2048, NULL, 1, NULL);
    xTaskCreate(&task_control_threshold_button, "task_control_threshold_button", 2048, NULL, 1, NULL);

}





//***********************************************************************FUNCIONES DEL ADC*********************************************/
int encontrar_posicion_mas_cercana(float* vector, int longitud, float variable) 
{
    int indice = 0;
    float distancia_minima = abs(vector[0] - variable);
    
    for (int i = 1; i < longitud; i++) {
        float distancia_actual = abs(vector[i] - variable);
        if (distancia_actual < distancia_minima) {
            indice = i;
            distancia_minima = distancia_actual;
        }
    }
    
    return indice;
}


//***********************************************************************FUNCIONES DEL UART**********************************************************
/****************************************Función para obtener el número del UART***************/
static int Get_number(char *data)
{
	uint32_t result;

	for (int i = 0; i<lenght_Rx + 1; i++){
		if (data[i] == '$'){
			pos_in = i;
			for (int j = i+1; j<lenght_Rx+1; j++){
				if (data[j] == '$'){
					pos_fin = j;
					break;
				}
			}
			break;
		}
	}


	result = 0;
	for (int z = pos_in+1; z<=pos_fin-1; z++ ){ 
		result += (data[z]-48)*(pow(10, pos_fin-1-z));
	}
    //printf("%d\n",result);

	return result;

}

/****************************************Función para IMPRIMIR el THRESHOLD***************/
static void print_Threshold(void){
    len = sprintf((char *)&message_to_send[0], "El THRESHOLD es: %d \r\n",threshold_temp);
    uart_write_bytes(UART_NUM, (const char*) message_to_send, len);
}

/****************************************Función para IMPRIMIR el HISTERESIS***************/
static void print_Hist(void){
    len = sprintf((char *)&message_to_send[0], "LA HISTERESIS es: %d \r\n",hist_temp);
    uart_write_bytes(UART_NUM, (const char*) message_to_send, len);
}

//***********************************************************************FUNCIONES DE INTERRUPCIONES**********************************************************/
/*Interrupción para incrementar el THRESHOLD*/
void isr_handlerSUM(void *args)
{
    banderaSUM = 1;
}


/*Interrupción para decrementar el THRESHOLD*/
void isr_handlerRES(void *args)
{
    banderaRES = 1;
}