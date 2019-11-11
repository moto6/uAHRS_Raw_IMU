#include "arduino.h"

_Serial Serial;

void arduinoInit(){
	// RTC 초기화
	rtc_init();
}

void _Serial::begin(uint32_t buadrate){
	// TX/RX 핀 설정
	nrf_uart_txrx_pins_set(NRF_UART0, UART_TX_PIN, UART_RX_PIN);

	// 보드 레이트 설정
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

	// 옵션 설정
	nrf_uart_configure(NRF_UART0, NRF_UART_PARITY_EXCLUDED, NRF_UART_HWFC_DISABLED);

	// UART 활성화
	nrf_uart_enable(NRF_UART0);

	// UART 시작
	nrf_uart_task_trigger(NRF_UART0, NRF_UART_TASK_STARTRX);
	nrf_uart_task_trigger(NRF_UART0, NRF_UART_TASK_STARTTX);
}

void _Serial::end(){
	// UART 활성화
	nrf_uart_disable(NRF_UART0);
}

void _Serial::print(const char *texts){
	// 아두이노의 println 함수
	// 전송할 데이터 길이 저장
	uint16_t len = strlen(texts);

	// UART로 데이터 보냄
	for(uint8_t i=0;i<len;i++){
		nrf_uart_txd_set(NRF_UART0, texts[i]);
		while(nrf_uart_event_check(NRF_UART0, NRF_UART_EVENT_TXDRDY) != 1){}
		nrf_uart_event_clear(NRF_UART0, NRF_UART_EVENT_TXDRDY);
	}
}

void _Serial::println(const char *texts){
	// 아두이노의 println 함수
	// 전송할 데이터 길이 저장
	uint16_t len = strlen(texts);

	// UART로 데이터 보냄
	for(uint8_t i=0;i<len;i++){
		nrf_uart_txd_set(NRF_UART0, texts[i]);
		while(nrf_uart_event_check(NRF_UART0, NRF_UART_EVENT_TXDRDY) != 1){}
		nrf_uart_event_clear(NRF_UART0, NRF_UART_EVENT_TXDRDY);
	}

	// LF 및 CR 보냄
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
	// 매핑 알고리즘
	// 출력값 변수 선언
	int16_t output;

	// 비율(Ratio) 계산
	float R = (float)(max - min) / (float)(_max - _min);

	// 최소값을 바탕으로 비율 적용
	output = min + (float)val*R;

	// 최대/최소값을 넘어설 경우 값 제한
	if(output >= max){output = max;}
	else if(output < min){output = min;}

	// 계산값 출력
	return output;
}
void rtc_init(){
	// Real Time Clock 초기화 함수
	// Low Frequency 클럭 소스 설정
	nrf_clock_lf_src_set(NRF_CLOCK_LFCLK_Xtal);

	// Start Clock
	nrf_clock_task_trigger(NRF_CLOCK_TASK_LFCLKSTART);
	while(nrf_clock_lf_start_task_status_get() != NRF_CLOCK_START_TASK_TRIGGERED){}

	// Initialize RTC
	// Stop the RTC
	nrf_rtc_task_trigger(NRF_RTC2, NRF_RTC_TASK_STOP);

	// 프리스케일러 설정
	// 32로 설정할 경우 32768 / 32 = 1024
	nrf_rtc_prescaler_set(NRF_RTC2, 32);

	// 인터럽트 해제
	nrf_rtc_int_disable(NRF_RTC2, 0x00);

	// Clear Task
	nrf_rtc_task_trigger(NRF_RTC2, NRF_RTC_TASK_CLEAR);
	nrf_rtc_task_trigger(NRF_RTC2, NRF_RTC_TASK_START);
}


uint32_t millis(){
	// Arduino millis() 을 구현한 함수
	// RTC0 : Softdevice에서 이미 사용중
	// RTC1 : 왜인지 작동 안됨
	// RTC2 : 작동됨
	// 반환하기 위한 변수 선언. RTC 값은 uint32_t로 정의되어 있음
	static uint32_t output = 0;

	// Read Compare/Counter Register
	output = nrf_rtc_counter_get(NRF_RTC2);
	output = (float)output/(1000./1024.);

	// Clear
	return output;
}

void pinMode(uint8_t port, bool dir){
	// GPIO 핀의 방향을 설정하는 함수
	// dir == 1일 경우 OUTPUT으로 설정
	if(dir){
		nrf_gpio_cfg_output(port);
	}else{
		nrf_gpio_cfg_input(port, NRF_GPIO_PIN_NOPULL);
	}
}

uint16_t analogRead(nrf_saadc_input_t portNo){
	// 아날로그 값 읽는 함수
	// 매개변수 : 포트 넘버
	// 기본 변수를 선언합니다
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
	// SAADC 채널 초기화
	nrf_saadc_channel_init(channel, &aread);

	// SAADC 해상도 설정
	nrf_saadc_resolution_set(NRF_SAADC_RESOLUTION_12BIT);

	// 데이터 버퍼 설정
	nrf_saadc_value_t buf_adc;
	memset(&buf_adc, 0x00, sizeof(buf_adc));
	nrf_saadc_buffer_init(&buf_adc, 1);

	// Oversampling 비활성화
	nrf_saadc_oversample_set(NRF_SAADC_OVERSAMPLE_DISABLED);

	// 연속모드 비활성화
	nrf_saadc_continuous_mode_disable();

	// SAADC 활성화
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

	// 버퍼에 저장된 데이터 값을 읽은 후 리턴합니다.
	return buf_adc;
}

void digitalWrite(uint8_t port, bool value){
	// 아두이노의 digitalWrite 함수
	if(value){
		nrf_gpio_pin_set(port);
	}else{
		nrf_gpio_pin_clear(port);
	}
}
