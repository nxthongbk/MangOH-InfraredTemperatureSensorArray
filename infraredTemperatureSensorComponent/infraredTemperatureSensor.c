#include "legato.h"
#include "interfaces.h"

#define TEMPERATURE_DATAHUB	"infraredTemperature/pixel"
double pixels_temp[64];

static void Temperature_pixel()
{
	size_t PIXEL_NUM = 64;
	ma_infraredTemperature_read_pixel_temperature(pixels_temp, &PIXEL_NUM);
}

COMPONENT_INIT
{
	le_result_t result;
	
	// This will be provided to the Data Hub. --> send to ledmatrix
	result = io_CreateInput(TEMPERATURE_DATAHUB,
				IO_DATA_TYPE_JSON,
				"Tempearture");
	LE_ASSERT(result == LE_OK);

	result = admin_SetSource("/app/infraredTemperatureDisplay/" TEMPERATURE_DATAHUB,
				 "/app/infraredTemperatureSensor/" TEMPERATURE_DATAHUB);
	LE_ASSERT(result == LE_OK);

	char json_str[IO_MAX_STRING_VALUE_LEN];
	while(1) {
		memset(json_str, 0 , IO_MAX_STRING_VALUE_LEN);
		Temperature_pixel();
		for (int i = 0; i < 64; i++) {
			sprintf(json_str,
				"{\"index\":%d,\"value\":%0.2f}", i, pixels_temp[i]);
			io_PushJson(TEMPERATURE_DATAHUB, IO_NOW, (const char *)json_str);
		}
		sleep(1);

	}
	
}
