#include "arduino.h"

_Serial Serial;

void arduinoInit(){
	// RTC �ʱ�ȭ
	rtc_init();
}

void _Serial::begin(uint32_t buadrate){
	// TX/RX �� ����
	nrf_uart_txrx_pins_set(NRF_UART0, UART_TX_PIN, UART_RX_PIN);

	// ���� ����Ʈ ����
	switch(buadrate){
	case 9600:
		nrf_uart_baudrate_set(NRF_UART0, NRF_UART_BAUDRATE_9600);
		break;
	case 19200:
		nrf_uart_baudrate_set(NRF_UART0, NRF_UART_BAUDRATE_19200);
		break;
	case 38400:
		nrf_uart_baudrate_set(NRF_UART0, NRF_UART_BAUDRATE_38400);
		break;
	case 57600:
		nrf_uart_baudrate_set(NRF_UART0, NRF_UART_BAUDRATE_57600);
		break;
	case 115200:
		nrf_uart_baudrate_set(NRF_UART0, NRF_UART_BAUDRATE_115200);
		break;
	case 230400:
		nrf_uart_baudrate_set(NRF_UART0, NRF_UART_BAUDRATE_230400);
		break;
	case 460800:
		nrf_uart_baudrate_set(NRF_UART0, NRF_UART_BAUDRATE_460800);
		break;
	case 921600:
		nrf_uart_baudrate_set(NRF_UART0, NRF_UART_BAUDRATE_921600);
		break;
	default:
		nrf_uart_baudrate_set(NRF_UART0, NRF_UART_BAUDRATE_115200);
		break;
	}

	// �ɼ� ����
	nrf_uart_configure(NRF_UART0, NRF_UART_PARITY_EXCLUDED, NRF_UART_HWFC_DISABLED);

	// UART Ȱ��ȭ
	nrf_uart_enable(NRF_UART0);

	// UART ����
	nrf_uart_task_trigger(NRF_UART0, NRF_UART_TASK_STARTRX);
	nrf_uart_task_trigger(NRF_UART0, NRF_UART_TASK_STARTTX);
}

void _Serial::end(){
	// UART Ȱ��ȭ
	nrf_uart_disable(NRF_UART0);
}

void _Serial::print(const char *texts){
	// �Ƶ��̳��� println �Լ�
	// ������ ������ ���� ����
	uint16_t len = strlen(texts);

	// UART�� ������ ����
	for(uint8_t i=0;i<len;i++){
		nrf_uart_txd_set(NRF_UART0, texts[i]);
		while(nrf_uart_event_check(NRF_UART0, NRF_UART_EVENT_TXDRDY) != 1){}
		nrf_uart_event_clear(NRF_UART0, NRF_UART_EVENT_TXDRDY);
	}
}

void _Serial::println(const char *texts){
	// �Ƶ��̳��� println �Լ�
	// ������ ������ ���� ����
	uint16_t len = strlen(texts);

	// UART�� ������ ����
	for(uint8_t i=0;i<len;i++){
		nrf_uart_txd_set(NRF_UART0, texts[i]);
		while(nrf_uart_event_check(NRF_UART0, NRF_UART_EVENT_TXDRDY) != 1){}
		nrf_uart_event_clear(NRF_UART0, NRF_UART_EVENT_TXDRDY);
	}

	// LF �� CR ����
	nrf_uart_txd_set(NRF_UART0, 10);
	while(nrf_uart_event_check(NRF_UART0, NRF_UART_EVENT_TXDRDY) != 1){}
	nrf_uart_event_clear(NRF_UART0, NRF_UART_EVENT_TXDRDY);
	nrf_uart_txd_set(NRF_UART0, 13);
	while(nrf_uart_event_check(NRF_UART0, NRF_UART_EVENT_TXDRDY) != 1){}
	nrf_uart_event_clear(NRF_UART0, NRF_UART_EVENT_TXDRDY);
}

uint8_t _Serial::read(){
	uint8_t output = 0;
	if(nrf_uart_event_check(NRF_UART0, NRF_UART_EVENT_RXDRDY) > 0){
		output = nrf_uart_rxd_get(NRF_UART0);
		nrf_uart_event_clear(NRF_UART0, NRF_UART_EVENT_RXDRDY);
	}
	return output;
}

void delay(uint16_t ms){
	nrf_delay_ms(ms);
}

int32_t constrain(int32_t val, int32_t min, int32_t max){
	if(val < min){val = min;}
	if(val > max){val = max;}
	return val;
}
int16_t map(int16_t val, int32_t _min, int32_t _max, int32_t min, int32_t max){
	// ���� �˰���
	// ��°� ���� ����
	int16_t output;

	// ����(Ratio) ���
	float R = (float)(max - min) / (float)(_max - _min);

	// �ּҰ��� �������� ���� ����
	output = min + (float)val*R;

	// �ִ�/�ּҰ��� �Ѿ ��� �� ����
	if(output >= max){output = max;}
	else if(output < min){output = min;}

	// ��갪 ���
	return output;
}
void rtc_init(){
	// Real Time Clock �ʱ�ȭ �Լ�
	// Low Frequency Ŭ�� �ҽ� ����
	nrf_clock_lf_src_set(NRF_CLOCK_LFCLK_Xtal);

	// Start Clock
	nrf_clock_task_trigger(NRF_CLOCK_TASK_LFCLKSTART);
	while(nrf_clock_lf_start_task_status_get() != NRF_CLOCK_START_TASK_TRIGGERED){}

	// Initialize RTC
	// Stop the RTC
	nrf_rtc_task_trigger(NRF_RTC2, NRF_RTC_TASK_STOP);

	// ���������Ϸ� ����
	// 32�� ������ ��� 32768 / 32 = 1024
	nrf_rtc_prescaler_set(NRF_RTC2, 32);

	// ���ͷ�Ʈ ����
	nrf_rtc_int_disable(NRF_RTC2, 0x00);

	// Clear Task
	nrf_rtc_task_trigger(NRF_RTC2, NRF_RTC_TASK_CLEAR);
	nrf_rtc_task_trigger(NRF_RTC2, NRF_RTC_TASK_START);
}


uint32_t millis(){
	// Arduino millis() �� ������ �Լ�
	// RTC0 : Softdevice���� �̹� �����
	// RTC1 : ������ �۵� �ȵ�
	// RTC2 : �۵���
	// ��ȯ�ϱ� ���� ���� ����. RTC ���� uint32_t�� ���ǵǾ� ����
	static uint32_t output = 0;

	// Read Compare/Counter Register
	output = nrf_rtc_counter_get(NRF_RTC2);
	output = (float)output/(1000./1024.);

	// Clear
	return output;
}

void pinMode(uint8_t port, bool dir){
	// GPIO ���� ������ �����ϴ� �Լ�
	// dir == 1�� ��� OUTPUT���� ����
	if(dir){
		nrf_gpio_cfg_output(port);
	}else{
		nrf_gpio_cfg_input(port, NRF_GPIO_PIN_NOPULL);
	}
}

uint16_t analogRead(nrf_saadc_input_t portNo){
	// �Ƴ��α� �� �д� �Լ�
	// �Ű����� : ��Ʈ �ѹ�
	// �⺻ ������ �����մϴ�
	int16_t output;
	uint8_t channel = 0;
	switch(portNo){
	case NRF_SAADC_INPUT_AIN0:
		channel = 0;
		break;
	case NRF_SAADC_INPUT_AIN1:
		channel = 1;
		break;
	case NRF_SAADC_INPUT_AIN2:
		channel = 2;
		break;
	case NRF_SAADC_INPUT_AIN3:
		channel = 3;
		break;
	case NRF_SAADC_INPUT_AIN4:
		channel = 4;
		break;
	case NRF_SAADC_INPUT_AIN5:
		channel = 5;
		break;
	case NRF_SAADC_INPUT_AIN6:
		channel = 6;
		break;
	case NRF_SAADC_INPUT_AIN7:
		channel = 7;
		break;
	default:
		channel = 0;
		break;
	}
	nrf_saadc_channel_config_t aread = {
			.resistor_p = NRF_SAADC_RESISTOR_DISABLED,
			.resistor_n = NRF_SAADC_RESISTOR_DISABLED,
			.gain = NRF_SAADC_GAIN1_6,
			.reference = NRF_SAADC_REFERENCE_INTERNAL,
			.acq_time = NRF_SAADC_ACQTIME_10US,
			.mode = NRF_SAADC_MODE_SINGLE_ENDED,
			.burst = NRF_SAADC_BURST_DISABLED,
			.pin_p = portNo,
			.pin_n = NRF_SAADC_INPUT_DISABLED
	};
	// SAADC ä�� �ʱ�ȭ
	nrf_saadc_channel_init(channel, &aread);

	// SAADC �ػ� ����
	nrf_saadc_resolution_set(NRF_SAADC_RESOLUTION_12BIT);

	// ������ ���� ����
	nrf_saadc_value_t buf_adc;
	memset(&buf_adc, 0x00, sizeof(buf_adc));
	nrf_saadc_buffer_init(&buf_adc, 1);

	// Oversampling ��Ȱ��ȭ
	nrf_saadc_oversample_set(NRF_SAADC_OVERSAMPLE_DISABLED);

	// ���Ӹ�� ��Ȱ��ȭ
	nrf_saadc_continuous_mode_disable();

	// SAADC Ȱ��ȭ
	nrf_saadc_enable();
	while(!nrf_saadc_enable_check()){}

	// TASK Start
	nrf_saadc_task_trigger(NRF_SAADC_TASK_START);
	while(!nrf_saadc_event_check(NRF_SAADC_EVENT_STARTED)){}
	nrf_saadc_event_clear(NRF_SAADC_EVENT_STARTED);

	// TASK Trigger
	nrf_saadc_task_trigger(NRF_SAADC_TASK_SAMPLE);
	while(!nrf_saadc_event_check(NRF_SAADC_EVENT_RESULTDONE)){}
	nrf_saadc_event_clear(NRF_SAADC_EVENT_RESULTDONE);

	// TASK Stop
	nrf_saadc_task_trigger(NRF_SAADC_TASK_STOP);
	while(!nrf_saadc_event_check(NRF_SAADC_EVENT_STOPPED)){}
	nrf_saadc_event_clear(NRF_SAADC_EVENT_STOPPED);

	// ���ۿ� ����� ������ ���� ���� �� �����մϴ�.
	return buf_adc;
}

void digitalWrite(uint8_t port, bool value){
	// �Ƶ��̳��� digitalWrite �Լ�
	if(value){
		nrf_gpio_pin_set(port);
	}else{
		nrf_gpio_pin_clear(port);
	}
}
